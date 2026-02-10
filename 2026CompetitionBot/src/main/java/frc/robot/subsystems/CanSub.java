// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CanSub extends SubsystemBase {
  byte m_data_buffer[];

  int m_ARBID;
  static int m_sensor1;
  static int m_sensor2;

  /** Creates a new CanSub. */

  //new isEmptySensor
  //new isFullSensor
  //new isFuelInEscalatorSensor

  public CanSub(int CustomSensorID) {
    m_data_buffer = new byte[8];

    m_ARBID = createCANId(0x123, CustomSensorID, 8, 10);
  }

  @Override
  public void periodic() {
    UpdateCustomSensor();
    // This method will be called once per scheduler run
  }

  public byte getDataBufferByte(int byteIndex) {
    return m_data_buffer[byteIndex];
  }

  // A helper function to assemble a full arbitration ID.
  public static int createCANId(int apiId, int deviceId, int manufacturer, int deviceType) {
    return ((int) (deviceType) & 0x1F) << 24 | ((int) (manufacturer) & 0xFF) << 16 | (apiId & 0x3FF) << 6
        | (deviceId & 0x3F);
  }

  public void UpdateCustomSensor() {
    ByteBuffer targetedMessageID = ByteBuffer.allocateDirect(4);//Must be direct
    targetedMessageID.order(ByteOrder.LITTLE_ENDIAN); //Set order of bytes
    targetedMessageID.asIntBuffer().put(0, m_ARBID); //Put the arbID into the buffer
    ByteBuffer timeStamp = ByteBuffer.allocateDirect(4); //Allocate memory for time stamp

    //byte[] output_data = new byte[8];

    try {
      // Read data from the CAN bus.  If this fails, fall back to the error handler.

      // FRCNetCommCANSessionMuxReceiveMessage
      // IntBuffer messageID - The combination of device type, manf, API ID, and device number. This 
      //                       will be replaced with the actual ID received (if the mask didn't require
      //                       an exact match).
      // int messageIDMask - Search for messages matching the messageID. A mask of 0xFFFFFFFF will only
      //                     match exact messages. A mask of 0xFFFFFFFC0 will match everything except
      //                     device ID allowing one class to process all sensors.
      // ByteBuffer timeStamp - The timestamp of when the message was received. Unsure if this is the
      //                        OS clock or a hardware clock linked to the CAN Bus processing.

      System.arraycopy(
          CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetedMessageID.asIntBuffer(), 0xFFFFFFC0, timeStamp), 0,
          m_data_buffer, 0, m_data_buffer.length);

      //m_do0.set(!m_do0.get());

      if((m_data_buffer != null) && (timeStamp != null)) {
        if(m_data_buffer.length >= 2) {
          int msb = m_data_buffer[0];
          if(msb < 0) {
            msb = msb + 256;
          }

          int lsb = m_data_buffer[1];
          if(lsb < 0) {
            lsb = lsb + 256;
          }

          m_sensor1 = (msb << 8) + lsb;
          System.out.println(m_sensor1);

          if(m_data_buffer.length >= 4) {

            int msb1 = m_data_buffer[2];
            if(msb1 < 0) {
              msb1 = msb1 + 256;
            }

            int lsb1 = m_data_buffer[3];
            if(lsb1 < 0) {
              lsb1 = lsb1 + 256;
            }

            m_sensor2 = (msb1 << 8) + lsb1;
            System.out.println(m_sensor2);
          }

        }

        //Send a message back to the same device
        // Note that NO REPEAT means it will only send the output once. 
        //CANJNI.FRCNetCommCANSessionMuxSendMessage(targetedMessageID.getInt(), output_data, CANJNI.CAN_SEND_PERIOD_NO_REPEAT); 


      }

    } catch (edu.wpi.first.hal.can.CANMessageNotFoundException e) {
      return;
      //No CAN message, not a bad thing due to periodicity of messages
    } catch (Exception e) {
      //Other exception, print it out to make sure user sees it
      System.out.println(e.toString());
    }
    return;
  }

  public int getSensor1() {
    return m_sensor1;
  }

  public int getSensor2() {
    return m_sensor2;
  }

  public boolean isHopperEmpty() {
    return false;
  }

  public boolean isHopperFull() {
    return false;
  }

  public boolean isFuelInEscalator() {
    return false;
  }
}
