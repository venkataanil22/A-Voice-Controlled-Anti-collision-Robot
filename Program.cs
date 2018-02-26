//------------------------------------------------------------------------------
// <copyright file="Program.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------
// This module provides sample code used to demonstrate the use
//	of the KinectAudioSource for speech recognition
//Modifed Version of program for ECE 544 Lunar Module Final Project 
//	Randon Stasney, Aditya Pawar, Venkata Anil Viswanadha, Brandon Biodrowski
// Version 1.1
// Modified command structure and set up to write to Wixel wireless.
// Determined output representations, chose serial string.


namespace Speech
{
    using System;
    using System.IO;
    using System.IO.Ports;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;
    using System.Threading;
    using Microsoft.Kinect;
    using Microsoft.Speech.AudioFormat;
    using Microsoft.Speech.Recognition;


    public class Program
    {

        public static void Main(string[] args)
        {
			// Detect th kinect on the computer
            KinectSensor sensor = (from sensorToCheck in KinectSensor.KinectSensors where sensorToCheck.Status == KinectStatus.Connected select sensorToCheck).FirstOrDefault();
            if (sensor == null)
            {
                Console.WriteLine(
                        "No Kinect sensors are attached to this computer or none of the ones that are\n" +
                        "attached are \"Connected\".\n" +
                        "Attach the KinectSensor and restart this application.\n" +
                        "If that doesn't work run SkeletonViewer-WPF to better understand the Status of\n" +
                        "the Kinect sensors.\n\n" +
                        "Press any key to continue.\n");

                // Give a chance for user to see console output before it is dismissed
                Console.ReadKey(true);
                return;
            }

            sensor.Start();

            // Obtain the KinectAudioSource to do audio capture
            KinectAudioSource source = sensor.AudioSource;
            source.EchoCancellationMode = EchoCancellationMode.None; // No AEC for this sample
            source.AutomaticGainControlEnabled = false; // Important to turn this off for speech recognition

            RecognizerInfo ri = GetKinectRecognizer();

            if (ri == null)
            {
                Console.WriteLine("Could not find Kinect speech recognizer. Please refer to the sample requirements.");
                return;
            }
					
            Console.WriteLine("Using: {0}", ri.Name);

            // NOTE: Need to wait 4 seconds for device to be ready right after initialization
            int wait = 4;
            while (wait > 0)
            {
                Console.Write("Device will be ready for speech recognition in {0} second(s).\r", wait--);
                Thread.Sleep(1000);
            }

            using (var sre = new SpeechRecognitionEngine(ri.Id))
            {
                //Here are choices of commands to use
                var commands = new Choices();
                commands.Add("right turn");
                commands.Add("left turn");
                commands.Add("slow");
                commands.Add("fast");
                commands.Add("stop");
                commands.Add("reverse");
                commands.Add("light");


                var gb = new GrammarBuilder { Culture = ri.Culture };

                // Specify the culture to match the recognizer in case we are running in a different culture.                                 
                gb.Append(commands);

                // Create the actual Grammar instance, and then load it into the speech recognizer.
                var g = new Grammar(gb);

                sre.LoadGrammar(g);
                sre.SpeechRecognized += SreSpeechRecognized;
                sre.SpeechRecognitionRejected += SreSpeechRecognitionRejected;

                using (Stream s = source.Start())
                {
                    sre.SetInputToAudioStream(
                        s, new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));

                    //Prints out the commands that needs to be talked.
                    Console.WriteLine("Recognizing speech. Say:'right turn', 'left turn',  'slow', 'fast', 'stop', 'reverse', 'light'");
                    sre.RecognizeAsync(RecognizeMode.Multiple);
                    Console.ReadLine();
                    Console.WriteLine("Stopping recognizer ...");
                    sre.RecognizeAsyncStop();
                }
            }

            sensor.Stop();
        }

        private static RecognizerInfo GetKinectRecognizer()
        {
            Func<RecognizerInfo, bool> matchingFunc = r =>
            {
                string value;
                r.AdditionalInfo.TryGetValue("Kinect", out value);
                return "True".Equals(value, StringComparison.InvariantCultureIgnoreCase) && "en-US".Equals(r.Culture.Name, StringComparison.InvariantCultureIgnoreCase);
            };
            return SpeechRecognitionEngine.InstalledRecognizers().Where(matchingFunc).FirstOrDefault();
        }

        //If the voice is not loud enough or voice does not exist in voice commands, the program will just throw the error message.
        private static void SreSpeechRecognitionRejected(object sender, SpeechRecognitionRejectedEventArgs e)
        {
            Console.WriteLine("\nSpeech Rejected");
            if (e.Result != null)
            {
                Console.WriteLine("Incorrect Entry");

            }
        }


        // Function that sends voice commands to arduino.
        // Voice has to be clear when you say the commands
		// Can adjust confidence required if runnig into recognition errors

        public static void SreSpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
			  // Set up the com port to the wireless wixel connected to Arduino
              SerialPort send = new SerialPort("COM12", 9600);

            if (e.Result.Confidence >= 0.7)
            {
                Console.WriteLine("\nSpeech Recognized: \t{0}\tConfidence:\t{1}", e.Result.Text, e.Result.Confidence);
                if (e.Result.Text == "right turn")
                {
                    Console.WriteLine(1);
                    send.Open();
                    send.WriteLine("1");
                    send.Close();
                }

                else if (e.Result.Text == "left turn")
                {

                    Console.WriteLine(2);
                    send.Open();
                    send.WriteLine("2");
                    send.Close();
                }
                else if (e.Result.Text == "slow")
                {

                    Console.WriteLine(3);
                    send.Open();
                    send.WriteLine("3");
                    send.Close();
                }

                else if(e.Result.Text=="fast")
                {
                    Console.WriteLine(4);
                    send.Open();
                    send.WriteLine("4");
                    send.Close();
                }

                else if (e.Result.Text == "stop")
                {
                    Console.WriteLine(5);
                    send.Open();
                    send.WriteLine("5");
                    send.Close();
                }
                else if (e.Result.Text == "reverse")
                {
                    Console.WriteLine(6);
                    send.Open();
                    send.WriteLine("6");
                    send.Close();
                }
                else if (e.Result.Text == "light")
                {
                    Console.WriteLine(7);
                    send.Open();
                    send.WriteLine("7");
                    send.Close();
                }

            }
            else
            {
                Console.WriteLine("\nSpeech Recognized but confidence was too low: \t{0}", e.Result.Confidence);
                Console.WriteLine("Please try Again");
            }
        }
    }
}