#include "zfy_imu_node/imu_handle.hpp"


bool imu_handle::decode(imu_data &imudata, const uint8_t buffer[]){
    raw_data rawdata;
    rawdata.head = U8_TO_U16(buffer[0], buffer[1]);
    if(rawdata.head != 0xa5a5){
        return false;
    }
    rawdata.accx = U8_TO_U16(buffer[3], buffer[2]);
    rawdata.accy = U8_TO_U16(buffer[5], buffer[4]);
    rawdata.accz = U8_TO_I16(buffer[7], buffer[6]);
    rawdata.gyrx = U8_TO_U16(buffer[9], buffer[8]);
    rawdata.gyry = U8_TO_U16(buffer[11], buffer[10]);
    rawdata.gyrz = U8_TO_U16(buffer[13], buffer[12]);
    rawdata.euly = U8_TO_U16(buffer[15], buffer[14]);
    rawdata.eulp = U8_TO_U16(buffer[17], buffer[16]);
    rawdata.eulr = U8_TO_U16(buffer[19], buffer[18]);
    rawdata.time = U8_TO_U16(buffer[21], buffer[20]);
    rawdata.crc = U8_TO_U16(buffer[23], buffer[22]);

    if(int16_t((rawdata.gyrz+rawdata.euly+rawdata.eulp+rawdata.eulr)) != rawdata.crc){
        RCLCPP_WARN(rclcpp::get_logger("serial_imu"), "crc is error");
        return false;
    }
    RCLCPP_INFO_ONCE(rclcpp::get_logger("serial_imu"), "imu receive data Successful."); 
    
    rawtoimu(imudata, rawdata);

    /* RCLCPP_INFO_STREAM(rclcpp::get_logger("serial_imu"), "acc: "<<(float)imudata.acc[0]<<" "<<imudata.acc[1]
        <<" "<<imudata.acc[2]); */
    /* RCLCPP_INFO_STREAM(rclcpp::get_logger("serial_imu"), "gyr: "<<(float)imudata.gyr[0]<<" "<<imudata.gyr[1]
        <<" "<<imudata.gyr[2]); */

    return true;

}

void imu_handle::rawtoimu(imu_data &imudata, raw_data &rawdata){
    imudata.acc[0] = (float)rawdata.accx / 10920;
    imudata.acc[1] = (float)rawdata.accy / 10920;
    imudata.acc[2] = (float)rawdata.accz / 10920;
    imudata.gyr[0] = (float)rawdata.gyrx * 0.000175;
    imudata.gyr[1] = (float)rawdata.gyry * 0.000175;
    imudata.gyr[2] = (float)rawdata.gyrz * 0.000175;
    rpytoquat((float)(rawdata.eulr / 18000 * 3.1415926),
              (float)(rawdata.eulp / 18000 * 3.1415926),
              (float)(rawdata.euly / 18000 * 3.1415926),
              imudata.quat);
}

void imu_handle::rpytoquat(float r, float p, float y, float q[4]){
    float cr = std::cos(r * 0.5);
    float sr = std::sin(r * 0.5);
    float cp = std::cos(p * 0.5);
    float sp = std::sin(p * 0.5);
    float cy = std::cos(y * 0.5);
    float sy = std::sin(y * 0.5);
    
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy; 

}


