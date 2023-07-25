using System;
using System.IO;
using System.Threading.Tasks;
using Numpy;
using Python.Runtime;
using Python.Included;
using System.Collections.Generic;
using System.Threading;
using System.CodeDom;
using System.Text.RegularExpressions;
using System.Collections;
using System.Diagnostics;

namespace MicroscopeController.Classes
{

    public class ImageFromCamera
    {
        public readonly byte[] imageData;
        public readonly StackParams stackParams;
        public readonly int planeIndex;
        public ImageFromCamera(byte[] imageBytes, int planeIndex, StackParams stackParams)
        {
            imageData = imageBytes;
            this.planeIndex = planeIndex;
            this.stackParams = stackParams;
        }
    }
    public struct StackParams
    {
        public readonly Guid id;
        public readonly int timepoint;
        public readonly int specimen;
        public readonly int channel;
        public readonly int numPlanes;
        public readonly int numChannels;
        public readonly bool areChannelsInterleaved;
        public readonly int numImages;
        public readonly int imageWidth;
        public readonly int imageHeight;
        public readonly Type imageType;
        public readonly int anisotropyFactor;
        public StackParams(
            int timepoint,
            int specimen,
            bool areChannelsInterleaved,
            int channel,
            int numPlanes,
            int numChannels,
            int bitsPerPixel,
            int imageWidth,
            int imageHeight,
            int anisotropyFactor)
        {
            id = Guid.NewGuid();
            this.timepoint = timepoint;
            this.specimen = specimen;
            this.numPlanes = numPlanes;
            this.numChannels = numChannels;
            this.areChannelsInterleaved = areChannelsInterleaved;
            if (areChannelsInterleaved == true)
            {
                this.channel = -1; // when channels are interlieved saving in one stack
            }
            else
            {
                this.channel = channel;
            }
            numImages = numPlanes * numChannels;
            this.anisotropyFactor = anisotropyFactor;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            switch (bitsPerPixel)
            {
                case 8:
                    imageType = typeof(byte);
                    break;
                case 16:
                    imageType = typeof(UInt16);
                    break;
                case 24:
                    throw new NotImplementedException("RGB24 image saving as a stack has not been implemented yet.");
                default:
                    throw new FormatException("Recieved unknown BitsPerPixel value, could not determine image format");
            }
        }
    }

    public class StackSaver
    {
        public StackParams currentStackParams;

        private PyModule pyScope;
        public bool IsStackSavingActive = false;
        private bool hasPythonBeenInitialised = false;

        private readonly Queue<ImageFromCamera> imageQueue;
        private CancellationTokenSource cancellationTokenSource;
        private Task savingTask;

        private int internalPlaneIndex = 0;
        private ulong numFramesSaved = 0;
        public StackSaver()
        {
            imageQueue = new Queue<ImageFromCamera>();
            cancellationTokenSource = new CancellationTokenSource();
            savingTask = Task.Run(imageSavingWorker);
        }

        private void imageSavingWorker()
        {
            while (!cancellationTokenSource.Token.IsCancellationRequested)
            {
                ImageFromCamera image;
                lock (imageQueue)
                {
                    while (imageQueue.Count == 0)
                    {
                        Monitor.Wait(imageQueue, 50);
                        if (cancellationTokenSource.Token.IsCancellationRequested) { return; }
                    }
                    Console.WriteLine($"Queue is not empty {imageQueue.Count}");
                    image = imageQueue.Dequeue();

                    try
                    {
                        writePlane(image);
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"Error saving imageData '{image.stackParams}, {image.planeIndex}': {ex.Message}");
                    }
                }
            }
        }

        public void EnqueueImage(byte[] imageData, int planeIndex, ulong driverFrameCount)
        {

            bool successfullyWaitedForOrderToSave = WaitForCorrectFrameToSave(driverFrameCount).Result;
            Console.WriteLine($"Starting to enqueue image with planeIndex {planeIndex}");
            if (successfullyWaitedForOrderToSave == false)
            {
                Debug.WriteLine($"Failed to await the correct order to save the image!");
                return;
            }
            lock (imageQueue)
            {
                if (internalPlaneIndex != planeIndex)
                {
                    Debug.WriteLine(
                        $"Internal planeIndex {internalPlaneIndex} of StackSaver does not match recieved plane index {planeIndex}!" +
                        $"There must be an error in synchronisation.");
                    return;
                }
                imageQueue.Enqueue(new ImageFromCamera(imageData, planeIndex, currentStackParams));
                Monitor.Pulse(imageQueue);
                numFramesSaved++;
                internalPlaneIndex++;
            }
        }

        public void Stop()
        {
            cancellationTokenSource.Cancel();
            savingTask.Wait();
        }

        public void forceFinishStackSaving()
        {
            lock (imageQueue)
            {
                cancellationTokenSource.Cancel();
                imageQueue.Clear();
            }
        }

        private void resetStackSavingWorker()
        {
            cancellationTokenSource = new CancellationTokenSource();
            savingTask = Task.Run(imageSavingWorker);
        }

        public async Task<bool> initialize(ulong initialFrameCount)
        {
            numFramesSaved = initialFrameCount;
            resetStackSavingWorker();
            if (hasPythonBeenInitialised == true)
            {
                return true;
            }
            await Installer.SetupPython();
            PythonEngine.Initialize();
            np.arange(1); // Initializing Python environemnt by doing something with NumPy
            PythonEngine.BeginAllowThreads();
            using (Py.GIL())
            {
                pyScope = Py.CreateScope();
            }
            hasPythonBeenInitialised = true;
            return true;
        }

        public async Task<bool> installPythonAndPackages()
        {

            await Installer.TryInstallPip();
            await Installer.PipInstallModule("tifffile");
            await Installer.PipInstallModule("scikit-imageData");
            return true;
        }

        private async Task<bool> WaitForCorrectFrameToSave(ulong driverFrameCount)
        {
            CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
            CancellationToken cancellationToken = cancellationTokenSource.Token;

            if (numFramesSaved > driverFrameCount)
            {
                throw new InvalidOperationException(
                    $"Thread index is smaller than currect finished thread count! driverFrameCount:{driverFrameCount}, finished count:{numFramesSaved}");
            }

            Task<bool> checkingTask = Task.Run(async () =>
            {
                int elapsed = 0;
                while (true)
                {
                    if (numFramesSaved == driverFrameCount)
                    {
                        Debug.WriteLine($"Frame waiting: Waited for: {elapsed}," +
                            $" numFramesSaved: {numFramesSaved}, driverFrameCount: {driverFrameCount}");
                        return true;
                    }
                    //TimeSpan desiredDelay = TimeSpan.FromTicks((long)(Stopwatch.Frequency * 0.0001)); // 100 microseconds, for some reason is slower overall

                    await Task.Delay(1, cancellationToken);
                    elapsed += 1;

                    // Throw an exception if the timeout of 1 second is reached
                    cancellationToken.ThrowIfCancellationRequested();
                }
            }, cancellationToken);


            // Cancel the checking task after 1 second
            cancellationTokenSource.CancelAfter(1000);

            try
            {
                // Wait for the checking task to complete and return the result
                return await checkingTask;
            }
            catch (OperationCanceledException)
            {
                Debug.WriteLine("Reached a timeout while waiting for correct index!");
                return false; // Timeout occurred
            }
        }
        public bool startStackSaving(StackParams stack)
        {
            int finishStackSavingTimeout = 5000;
            while (IsStackSavingActive & imageQueue.Count > 0)
            {
                Task.Delay(50).Wait();
                finishStackSavingTimeout -= 50;
                if (finishStackSavingTimeout < 0)
                {
                    Debug.WriteLine($"Reached timeout while waiting for previous stack saving to finish.");
                    return false;
                }
            }
            if (IsStackSavingActive)
            {
                Debug.WriteLine("IsStackSavingActive is still true");
                return false;
            }
            Stopwatch sssTimer = new Stopwatch();
            sssTimer.Start();
            IsStackSavingActive = true;
            currentStackParams = stack;
            internalPlaneIndex = 0;
            using (Py.GIL())
            {
                pyScope.Set("num_planes", stack.numPlanes);
                pyScope.Set("x_width", stack.imageWidth);
                pyScope.Set("y_height", stack.imageHeight);

                if (stack.imageType == typeof(UInt16))
                {
                    pyScope.Set("image_type", np.uint16);
                }
                else if (stack.imageType == typeof(byte))
                {
                    pyScope.Set("image_type", np.uint8);
                }
                pyScope.Set("timepoint", stack.timepoint);
                pyScope.Set("anisotropy_factor", stack.anisotropyFactor);
                pyScope.Exec(
    @"
import tifffile
import numpy as np
import time
from skimage.transform import resize

image_type = image_type.get_PyObject()

omexml = tifffile.OmeXml()
omexml.addimage(
    dtype=image_type,
    shape=(num_planes, y_height, x_width),
    storedshape=(num_planes, 1, 1, y_height, x_width, 1),
    axes='ZYX',  # or DimensionOrder='XYZCT'
)
omexml = omexml.tostring()

tif = tifffile.TiffWriter(f's_out_{timepoint}.ome.tif', ome=False, bigtiff=True)
x_projection = np.empty((num_planes, x_width), dtype=image_type)
y_projection = np.empty((num_planes, y_height), dtype=image_type)
");
            }
            sssTimer.Stop();
            var toConfirmationSW_time = Math.Round(((double)sssTimer.ElapsedTicks / (double)Stopwatch.Frequency) * 1000, 2);
            Debug.WriteLine($"Starting stack saving took: {toConfirmationSW_time} ms");
            Debug.WriteLine($"Initialized stack saving for stack {stack.id}");
            return true;
        }

        public void writePlane(ImageFromCamera image)
        {
            Console.WriteLine($"Starting to save image {image.planeIndex}");
            using (Py.GIL())
            {
                NDarray numpyArray;
                if (image.stackParams.imageType == typeof(UInt16))
                {
                    Int16[] intArray = new Int16[image.imageData.Length / 2];
                    Buffer.BlockCopy(image.imageData, 0, intArray, 0, image.imageData.Length);
                    numpyArray = np.array(intArray).reshape(1, currentStackParams.imageHeight, currentStackParams.imageWidth);
                }
                else
                {
                    numpyArray = np.array(image.imageData).reshape(currentStackParams.imageHeight, currentStackParams.imageWidth);
                }
                Debug.WriteLine(numpyArray.shape.ToString());
                pyScope.Set("arrayCsharp", numpyArray);
                pyScope.Set("line_index", image.planeIndex);
                Debug.WriteLine($"Writing image plane: {image.planeIndex} timepoint: {image.stackParams.timepoint}");
                pyScope.Exec(
@"start_time = time.time()
pyArray = arrayCsharp.get_PyObject() # This allows you to access the true numpy array under the C# wrapper

plane = pyArray.astype(np.uint8)

tif.write(plane, description=omexml, contiguous=True)
print(plane.shape, x_projection.shape, ""Shapes"")
x_projection[line_index] = plane.max(axis=0)
y_projection[line_index] = plane.max(axis=1)
omexml = None
end_time = time.time()
execution_time = end_time - start_time

# Print the execution time
print(f'Execution time: {execution_time} seconds')"
                    );

            }
            if (image.planeIndex == image.stackParams.numImages - 1)
            {
                finishStackSaving();
            }
            if (image.planeIndex > image.stackParams.numImages - 1)
            {
                throw new Exception($"Recieved image planeIndex {image.planeIndex} " +
                    $"is higher than total number of planes in the stack {image.stackParams.numImages}!");
            }
            Console.WriteLine($"Finished saving image {image.planeIndex}");
        }

        private void finishStackSaving()
        {
            if (!IsStackSavingActive) return;
            using (Py.GIL())
            {
                pyScope.Exec(
    @"tif.close()
y_projection = resize(y_projection, (num_planes * anisotropy_factor, y_projection.shape[1]))
x_projection = resize(x_projection, (num_planes * anisotropy_factor, x_projection.shape[1]))



def convert(img, target_type_min, target_type_max, target_type):
    imin = img.min()
    imax = img.max()
    print(imin, imax, target_type_max)
    if imax == imin:
        a = target_type_max - target_type_min
    else:
        a = (target_type_max - target_type_min) / (imax - imin)
    b = target_type_max - a * imax
    new_img = (a * img + b).astype(target_type)
    return new_img

tifffile.imwrite('Y_projection_embryo.ome.tif', convert(
    y_projection, 0, np.iinfo('uint16').max, np.uint16), imagej=True)
tifffile.imwrite('X_projection_embryo.ome.tif', convert(
    x_projection, 0, np.iinfo('uint16').max, np.uint16), imagej=True)
");
            }
            IsStackSavingActive = false;
            Debug.WriteLine($"Successfully finished saving the stack.");
        }
    }
}
