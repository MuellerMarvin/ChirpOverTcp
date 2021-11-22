using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Collections.Generic;

class MyTcpListener
{
    public static void Main()
    {
        int fileNumber = 1;
        while(true)
        { 
            TcpListener server = null;
            try
            {
                // Set the TcpListener on port 13000.
                Int32 port = 13339;
                IPAddress localAddr = IPAddress.Parse("0.0.0.0");

                // TcpListener server = new TcpListener(port);
                server = new TcpListener(localAddr, port);

                // Start listening for client requests.
                server.Start();
                Byte[] bytes = new Byte[256];
                String data = null;
                // Enter the listening loop.
                while (true)
                {
                    Console.Write("Waiting for a connection... (Port: {0})", port);

                    // Perform a blocking call to accept requests.
                    // You could also use server.AcceptSocket() here.
                    TcpClient client = server.AcceptTcpClient();
                    Console.WriteLine("Connected!");
                    data = null;
                    // Get a stream object for reading and writing
                    NetworkStream stream = client.GetStream();
                    Directory.CreateDirectory("./audio_files");
                    StreamWriter localFile = new StreamWriter("./audio_files/Received_" + fileNumber + ".wav");


                    int i;
                    // Loop to receive all the data sent by the client.
                    Console.WriteLine("Receiving file {0}...", fileNumber);

                    // Loop to receive all the data sent by the client.
                    while ((i = stream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        // Translate data bytes to a ASCII string.
                        data = System.Text.Encoding.ASCII.GetString(bytes, 0, i);
                        // Process the data sent by the client.
                        // data = data.ToUpper();

                        byte[] msg = System.Text.Encoding.ASCII.GetBytes(data);

                        // Send back a response.
                        localFile.BaseStream.Write(msg, 0, msg.Length);
                    }


                    // Shutdown and end connection
                    client.Close();
                    localFile.Flush();
                    localFile.Close();
                    fileNumber++;
                }
            }
            catch (SocketException e)
            {
                Console.WriteLine("SocketException: {0}", e);
            }
            finally
            {
                // Stop listening for new clients.
                server.Stop();
            }
        }
    }
}