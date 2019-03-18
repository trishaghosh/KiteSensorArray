# KiteSensorArray
Final Code for the on-board Kite Sensors
This is the Final code for the On-board kite sensors. This code needs to be integrated with the TX code of the Radio Frequency Transmission to complete Tx side coding


//Sample Code for Display 

Display_Handle disp;
Display_Params params;

Display_init();

Display_Params_init(&params);
disp = Display_open(Display_Type_UART, &params);
Display_clear(disp);

Display_printf(disp,0,0, "Accelerometer Readings");
Display_printf(disp,0,0, "%f", Accel_Data[0]);   //X axis
Display_printf(disp,0,0, "%f", Accel_Data[1]);   //Y axis
Display_printf(disp,0,0, "%f", Accel_Data[2]);     //Z axis

Display_printf(disp,0,0, "Gyroscope Readings");
Display_printf(disp,0,0, "%f", Gyro_Data[0]);   //X Axis
Display_printf(disp,0,0, "%f", Gyro_Data[1]);   //Y Axis
Display_printf(disp,0,0, "%f",Gyro_Data[2]);    //Z Axis

Display_printf(disp,0,0, "Temperature Readings: %f", temperature);

Display_printf(disp,0,0, "Latitude: %f", lat);
Display_printf(disp,0,0, "Longitude %f", lon);
Display_printf(disp,0,0, "Altitude: %f", alt);
