// ------------- Quaternion ----------------------
VectorFloat toEuler(Quaternion q){
        double yaw;
        double pitch;
        double roll;
       
        double sinr = 2.0 * (q.w * q.x + q.y * q.z);
        double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = atan2(sinr, cosr);
      
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if(fabs(sinp)>=1){
          if(sinp>=0){
            pitch = PI/2;
          }else if (sinp<0){
            pitch = -PI/2;
          }
        }else{
          pitch = asin(sinp);
        }
      
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (sq(q.y) + sq(q.z));
        yaw = atan2(siny,cosy);
      
        return VectorFloat(yaw, pitch, roll);
}



Quaternion getDifference(Quaternion frameOfRefrence, Quaternion target){
        Quaternion diff = frameOfRefrence.getConjugate().getProduct(target);
        return diff;
}



Quaternion addToQuaternion(Quaternion q, float yaw, float pitch, float roll){ 
        Quaternion extraRotation = getQuaternion(yaw, pitch, roll);
        Quaternion result = q.getProduct(extraRotation);
        return result;
}



Quaternion getQuaternion(float yaw, float pitch, float roll){
        Quaternion newQuaternion;

        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
      
        
        newQuaternion.w = cr * cy * cp + sr * sy * sp;
        newQuaternion.z = cr * sy * cp - sr * cy * sp;
        newQuaternion.y = cr * cy * sp + sr * sy * cp;
        newQuaternion.x = sr * cy * cp - cr * sy * sp;
        return newQuaternion;
}
