// Hallym University
// Seung-Chan Kim
// Visualizer for Furniture Project
// 2018/01/02

import processing.serial.*;

Serial myPort;  // The serial port
int lf = 10;    // Linefeed in ASCII
String myString = null;

float[] keep_val1;

float[] keep_acc_x;
float[] keep_acc_y;
float[] keep_acc_z;

float[] keep_gyro_x;
float[] keep_gyro_y;
float[] keep_gyro_z;

float[] keep_button1;
float[] keep_sensor1;

float[] keep_mag_x;
float[] keep_mag_y;
float[] keep_mag_z;
int nSample = 200;

char[] dataPacket = new char[24];  
String dataString = "";
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

int accX_bit;
int accY_bit;
int accZ_bit;
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;

PrintWriter output;

boolean bRecord = false;
ArrayList <String> data;
long t0;
void setup() {
  
  keep_val1 = new float[nSample];
  keep_acc_x = new float[nSample];
  keep_acc_y = new float[nSample];
  keep_acc_z = new float[nSample];
  
  keep_gyro_x = new float[nSample];
  keep_gyro_y = new float[nSample];
  keep_gyro_z = new float[nSample];
  
  keep_mag_x = new float[nSample];
  keep_mag_y = new float[nSample];
  keep_mag_z = new float[nSample];
  
  keep_button1 = new float[nSample];
  keep_sensor1 = new float[nSample];
  
  
  for(int i=0; i<nSample; i++)
  {
    keep_val1[i] = 0.0;
    keep_acc_x[i] = 0.0;
    keep_acc_y[i] = 0.0;
    keep_acc_z[i] = 0.0;
    
    keep_gyro_x[i] = 0.0;
    keep_gyro_y[i] = 0.0;
    keep_gyro_z[i] = 0.0;
    
    keep_mag_x[i] = 0.0;
    keep_mag_y[i] = 0.0;
    keep_mag_z[i] = 0.0;
    
    keep_button1[i] = 0.0;
    keep_sensor1[i] = 0.0;
    
  }
  size(800, 600);
  
  String strOS = System.getProperty("os.name");
  println(strOS);
  
  // List all the available serial ports
  printArray(Serial.list());
  String portName;
  int portnum=0;
  if(System.getProperty("os.name").compareTo("Mac OS X") ==0)
    portnum = 1;
    
  portName = Serial.list()[portnum];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
 //   portName = "COM4";
    
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, portName, 38400);
  myPort.clear();
//  myPort.write('r');
  data = new ArrayList<String>();
  
  t0 = 0;
}

void keyPressed() {
  
  switch(key)
  {
  case 'a':
      myPort.write('a');
      break;
  case 's':
      myPort.write('s');
      break;
  case 'q':
      data.clear();
      bRecord = true;
      t0 = System.nanoTime();
      println("Began recording...");
      break;
  case 'w':
      bRecord = false;
      saveToFile();
      break;
  } 
}

void saveToFile()
{  
  int N = data.size();
  
  if(N > 0)
  {
    String dataName;
    if(System.getProperty("os.name").compareTo("Mac OS X") ==0)
    {
      //dataName= String.format("/Users/seung/repos/_outFurniture/Data_%2d_%2d_%2d_%2d_%2d.txt", month(), day(), hour(), minute(), second()); 
      dataName= String.format("/Users/seung/Dropbox/kimsc@disneyresearch.com/repos-out/_outFurniture/Data_%2d_%2d_%2d_%2d_%2d.txt", month(), day(), hour(), minute(), second()); 
    }
    else
    {
      //dataName= String.format("C:/work/_out/Data_%2d_%2d_%2d_%2d_%2d.txt", month(), day(), hour(), minute(), second()); 
      //dataName= String.format("C:/Users/seung/Dropbox/kimsc@disneyresearch.com/repos-out/_outFurniture/Data_%2d_%2d_%2d_%2d_%2d.txt", month(), day(), hour(), minute(), second());
      String path = sketchPath();
      dataName= String.format("%s/out/Data_%d%02d%02d_%02d%02d%02d.txt", path, year(), month(), day(), hour(), minute(), second());
      
    }
    
    output = createWriter(dataName);
    for(int ii=0; ii<N; ii++)
    {
      output.println(data.get(ii));
    }
    output.flush(); //<>//
    output.close();
    data.clear();
    String buf = String.format("Written %d samples", N); 
    println(dataName);
    println(buf);
  }
}


// See https://processing.org/discourse/beta/num_1254511350.html
void draw() {
//  if (millis() - interval > 1000) 
  {
        // resend single character to trigger DMP init/start
        // in case the MPU is halted/reset while applet is running
        
        background(255);  
        fill(0); 
        String buf;
        buf = String.format("Sensor : MPU-9250 (InvenSense)"); 
        text(buf, 50,50);
        buf = String.format("a : sensor start, q: record start, w: write to file");
        text(buf, 50,70);
        text(dataString, 50,90);
        buf = String.format("Data Vector Size: %d", data.size());
        text(buf, 50, 110);
        
        interval = millis();
        
        
        int offset_x1 = 100;
        int offset_y1 = 200;
        int offset_y2 = 400;
        
        int offset_y3 = 500;
        int offset_y4 = 600;

        float g = 10*3;
        float g2 = 0.1*3;
        float gx = 2;
        float g3 = 50;
        float g4 = 0.1;
        int y1, y2;
        
        
        for(int i=0; i<nSample-1; i++)
        {
          y1 = int(offset_y1 - g* keep_acc_x[i]) - 0;
          y2 = int(offset_y1 - g* keep_acc_x[i+1]) - 0;
          stroke(255,0,0);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          y1 = int(offset_y1 - g* keep_acc_y[i]) - 0;
          y2 = int(offset_y1 - g* keep_acc_y[i+1]) - 0;
          stroke(0,255,0);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          y1 = int(offset_y1 - g* keep_acc_z[i]) - 0;
          y2 = int(offset_y1 - g* keep_acc_z[i+1]) - 0;
          stroke(0,0,255);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          //////////// gyro //////////////
          
          y1 = int(offset_y2 - g2* keep_gyro_x[i]) - 0;
          y2 = int(offset_y2 - g2* keep_gyro_x[i+1]) - 0;
          stroke(255,0,0);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          y1 = int(offset_y2 - g2* keep_gyro_y[i]) - 0;
          y2 = int(offset_y2 - g2* keep_gyro_y[i+1]) - 0;
          stroke(0,255,0);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          y1 = int(offset_y2 - g2* keep_gyro_z[i]) - 0;
          y2 = int(offset_y2 - g2* keep_gyro_z[i+1]) - 0;
          stroke(0,0,255);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);
          
          
          //////////// Button //////////////
          y1 = int(offset_y3 - g3* keep_button1[i]) - 0;
          y2 = int(offset_y3 - g3* keep_button1[i+1]) - 0;
          
          stroke(64,64,64);
          line(offset_x1, offset_y3, offset_x1 + gx* nSample, offset_y3);
          
          stroke(255,0,0);
          line(offset_x1 + gx*i, y1, offset_x1 + gx*(i+1), y2);

          
        }
    }
    
}




void serialEvent(Serial port) {
//    println("EVENT");
    
    while (port.available() > 0) {
        int ch = myPort.read();
    //    String tempBuf = String.format("%d ", ch);
    //    print(tempBuf);
        
        if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
        synced = 1;
        //print ((char)ch);

        if ((serialCount == 1 && ch != 2)
            || (serialCount == 22 && ch != '\r')
            || (serialCount == 23 && ch != '\n'))  {
            serialCount = 0;
            synced = 0;
            return;
        }

        if (serialCount > 0 || ch == '$') {
            dataPacket[serialCount++] = (char)ch;
            if (serialCount == 24) {
                serialCount = 0; // restart packet byte position
                interval = millis();
                // get quaternion from data packet
                
                int accX_bit, accY_bit, accZ_bit;
                int gyroX_bit, gyroY_bit, gyroZ_bit;
                //int magX_bit, magY_bit, magZ_bit;
                int button1;
                int snd1;
                
                accX_bit = ((dataPacket[2] << 8) | dataPacket[3]);// * 2.0 / 32768.0f;
                accY_bit = ((dataPacket[4] << 8) | dataPacket[5]);// * 2.0 / 32768.0f;
                accZ_bit = ((dataPacket[6] << 8) | dataPacket[7]);// * 2.0 / 32768.0f;
                
                gyroX_bit = ((dataPacket[8] << 8) | dataPacket[9]);
                gyroY_bit = ((dataPacket[10] << 8) | dataPacket[11]);
                gyroZ_bit = ((dataPacket[12] << 8) | dataPacket[13]);
                
                //button1 = dataPacket[14];
                
                //snd1 =  (dataPacket[15] << 8) + dataPacket[16]; //vh * 512 + vl;
                
                //magX_bit = ((dataPacket[14] << 8) | dataPacket[15]);
                //magY_bit = ((dataPacket[16] << 8) | dataPacket[17]);
                //magZ_bit = ((dataPacket[18] << 8) | dataPacket[19]);
                
                accX = two_complement(accX_bit) * 2.0 / 32768.0f;
                accY = two_complement(accY_bit) * 2.0 / 32768.0f;
                accZ = two_complement(accZ_bit) * 2.0 / 32768.0f;
                
                gyroX = two_complement(gyroX_bit) * 250.0 / 32768.0f;
                gyroY = two_complement(gyroY_bit) * 250.0 / 32768.0f;
                gyroZ = two_complement(gyroZ_bit) * 250.0 / 32768.0f;
                
                //magX = two_complement(magX_bit) * 4800.0 / 16384.0f;
                //magY = two_complement(magY_bit) * 4800.0 / 16384.0f;
                //magZ = two_complement(magZ_bit) * 4800.0 / 16384.0f;
                
                if (bRecord == true)
                {
                    String dataBuf = String.format("%d\t%f\t%f\t%f\t%f\t%f\t%f",//\t%d\t%d
                    System.nanoTime(), accX, accY, accZ, gyroX, gyroY, gyroZ);//, button1, snd1
                    data.add(dataBuf);
                }
                ShiftWithNewf(keep_acc_x, accX);
                ShiftWithNewf(keep_acc_y, accY);
                ShiftWithNewf(keep_acc_z, accZ);
                
                ShiftWithNewf(keep_gyro_x, gyroX);
                ShiftWithNewf(keep_gyro_y, gyroY);
                ShiftWithNewf(keep_gyro_z, gyroZ);
                
                
                //ShiftWithNewf(keep_button1, button1);
                //ShiftWithNewf(keep_sensor1, snd1);
                
                

                dataString = String.format("%d (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f)", 
                interval, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ);
            //    println("All packet received");
                /*
                // below calculations unnecessary for orientation only using toxilibs
                
                // calculate gravity vector
                gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
                gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
                gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
                // calculate Euler angles
                euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
                euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
    
                // calculate yaw/pitch/roll angles
                ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
                ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
                ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
    
                // output various components for debugging
                //println("q:\t" + round(q[0]*100.0f)/100.0f + "\t" + round(q[1]*100.0f)/100.0f + "\t" + round(q[2]*100.0f)/100.0f + "\t" + round(q[3]*100.0f)/100.0f);
                //println("euler:\t" + euler[0]*180.0f/PI + "\t" + euler[1]*180.0f/PI + "\t" + euler[2]*180.0f/PI);
                //println("ypr:\t" + ypr[0]*180.0f/PI + "\t" + ypr[1]*180.0f/PI + "\t" + ypr[2]*180.0f/PI);
                */
            }
        }
    }
    
}

int two_complement(int value)
{
  if (value - 0x8000 > 0)
  {
      value = (int)(0x7FFF&value)-0x8000;
  }
  return value;
}


float GetMean(float[] arr)
{
  int sz = arr.length;
  int i;
  float sum =0;
  for(i=0; i< sz; i++)
  {
    sum = sum + arr[i];
  }
  
  float mean = sum / float(i);
  return mean;
}


void ShiftWithNewd(int[] arr, int n_val)
{
  int sz = arr.length;
  int i;
  for(i=0; i< sz-1; i++)
  {
    arr[sz-1-i] = arr[sz-2-i];
    
  }

  arr[0] = n_val;

}
void ShiftWithNewf(float[] arr, float n_val)
{
  int sz = arr.length;
  int i;
  for(i=0; i< sz-1; i++)
  {
    arr[sz-1-i] = arr[sz-2-i];
    
  }

  arr[0] = n_val;

}
