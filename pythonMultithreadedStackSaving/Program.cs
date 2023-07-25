using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Numpy;
using Python.Runtime;
using Python.Included;
using Numpy.Models;
using System;
using MicroscopeController.Classes;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.IO;
using System.Threading;

namespace MicroscopeController
{

    class Program
    {


        static int numberOfFinishedTasks = 0;

        private static int PlanesInTiff = 10;
        private static int XInTiff = 4096;
        private static int YInTiff = 3000;
        private static byte[] gradientArray = new byte[XInTiff * YInTiff];
        private static byte[] realImage = new byte[XInTiff * YInTiff];
        private static StackSaver stackSaver = new StackSaver();

        private static readonly object lockBigTiff = new object();
        static async Task Main(string[] args)
        {
            numberOfFinishedTasks = 0;


            for (int i = 0; i < XInTiff * YInTiff; i++)
            {
                gradientArray[i] = Convert.ToByte(Math.Round(Convert.ToDouble(i) / (XInTiff * YInTiff) * 100));
            }
            //string executableDirectory = AppDomain.CurrentDomain.BaseDirectory;
            string imagePath = Path.Combine(@"C:\Users\ak-stelzer\Documents\for_testing", "sample_embryo_image.bmp");
            if (File.Exists(imagePath))
            {
                realImage = Load8BitImageData(imagePath);

                Console.WriteLine($"Image size: {realImage.Length} bytes");
            }
            else
            {
                Console.WriteLine("Image file not found. Exiting the application.");
                Environment.Exit(1); // Use an appropriate exit code (non-zero) to indicate an error condition
            }

            //var success = stackSaver.installPythonAndPackages();
            stackSaver.initialize(0).Wait();
            stackSaver.startStackSaving(new StackParams(0, 0, false, 0, PlanesInTiff, 1, 8, XInTiff, YInTiff, 4));



            Stopwatch stopwatch = Stopwatch.StartNew();

            LaunchFireAndForgetTasks();

            await WaitForStackFinishedSaving(stackSaver);


            stopwatch.Stop();
            long elapsedTime = stopwatch.ElapsedMilliseconds;
            Console.WriteLine($"Total BigTIFF writing Time: {elapsedTime} ms");
        }

        static byte[] Load8BitImageData(string imagePath)
        {
            // Check if the file exists
            if (!System.IO.File.Exists(imagePath))
            {
                Console.WriteLine("Image file not found.");
                return null;
            }

            // Load the image from file
            Bitmap bmp = new Bitmap(imagePath);

            // Check if the image is 8-bit grayscale
            if (bmp.PixelFormat != PixelFormat.Format8bppIndexed)
            {
                Console.WriteLine("The image is not an 8-bit grayscale image.");
                return null;
            }

            // Get the image width and height
            int width = bmp.Width;
            int height = bmp.Height;

            // Create a byte array to hold all pixel values
            byte[] pixelData = new byte[width * height];

            // Lock the image data to get direct access to the pixels
            BitmapData bmpData = bmp.LockBits(new Rectangle(0, 0, width, height), ImageLockMode.ReadOnly, bmp.PixelFormat);

            try
            {
                // Copy the pixel data from the image to the byte array
                Marshal.Copy(bmpData.Scan0, pixelData, 0, pixelData.Length);
            }
            finally
            {
                // Unlock the image data
                bmp.UnlockBits(bmpData);
            }

            // Return the byte array containing the pixel values
            return pixelData;
        }

        static void LaunchFireAndForgetTasks()
        {
            for (int i = 0; i < PlanesInTiff; i++)
            {
                int lineIndex = i; // Capture the loop variable

                Task.Run(() =>
                {
                    stackSaver.EnqueueImage(realImage, lineIndex, (ulong)lineIndex);
                    numberOfFinishedTasks++;
                });
            }
        }

        static async Task<bool> CheckFinishedAsync()
        {
            // Start a timer to check the value every 10 ms
            CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
            CancellationToken cancellationToken = cancellationTokenSource.Token;

            Task<bool> checkingTask = Task.Run(async () =>
            {
                int elapsed = 0;
                while (true)
                {
                    if (numberOfFinishedTasks >= PlanesInTiff)
                    {
                        Console.WriteLine($"Waited for: {elapsed}");
                        return true;
                    }

                    await Task.Delay(10, cancellationToken);
                    elapsed += 10;

                    // Throw an exception if the timeout of 1 second is reached
                    cancellationToken.ThrowIfCancellationRequested();
                }
            }, cancellationToken);


            // Cancel the checking task after 1 second
            cancellationTokenSource.CancelAfter(10000);

            try
            {
                // Wait for the checking task to complete and return the result
                return await checkingTask;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("Reached a timeout while waiting for all Tasks to finish!");
                return false; // Timeout occurred
            }
        }

        static async Task<bool> WaitForStackFinishedSaving(StackSaver stackSaver)
        {
            // Start a timer to check the value every 1 ms
            CancellationTokenSource cancellationTokenSource = new CancellationTokenSource();
            CancellationToken cancellationToken = cancellationTokenSource.Token;

            //if (numberOfFinishedTasks > thread_index)
            //{
            //    throw new InvalidOperationException(
            //        $"Thread index is smaller than currect finished thread count! thread_index:{thread_index}, finished count:{numberOfFinishedTasks}");
            //}

            Task<bool> checkingTask = Task.Run(async () =>
            {
                int elapsed = 0;
                while (true)
                {
                    if (stackSaver.IsStackSavingActive == false)
                    {
                        Console.WriteLine($"Waited for: {elapsed}");
                        return true;
                    }

                    await Task.Delay(2, cancellationToken);
                    elapsed += 2;

                    cancellationToken.ThrowIfCancellationRequested();
                }
            }, cancellationToken);


            cancellationTokenSource.CancelAfter(10000);

            try
            {
                // Wait for the checking task to complete and return the result
                return await checkingTask;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("Reached a timeout while waiting for stack to finish saving!");
                return false; // Timeout occurred
            }
        }
    }

}