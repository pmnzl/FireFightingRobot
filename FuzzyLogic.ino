/* 
 *  Fuzzy constants/thresholds. All distances in mm
 */

/* 
 *  Avoid obstacle behaviour
 */


float dist_target = 100; //100
/*
 * Fuzzification: Fuzzy Membership functions
 */
// Membership funciton for distance - close
float fs_close(float dist){ 
  if(dist < dist_target){ 
     return (1.0);
  }else if(dist >= dist_target){ 
    return 0.0;
  //}else{ 
//    return (2 - dist/50);
  }  
}

// Membership funciton for distance - far
float fs_far(float dist){ 
  if(dist < dist_target){ 
     return (0.0);
  }else if(dist >= dist_target){ 
    return (1.0);
//  }else{ 
//    return (dist/50 - 1);
  }  
}

/* 
 *  Fuzzy contoller for co-ordinating other helper funcitons and 
 *  for inference and aggregation 
 */
float fuzzy_controller(float dl, float dc, float dr, float dl_rear, float dr_rear){
  // Define output membership functions
  float dir_left = 0;
  float dir_right = 0; 
  float dir_fwd = 0; 
  
  //If dc is FAR then dir is FWD
  dir_fwd = max(fs_far(dc),dir_fwd);
  
  //If dc is CLOSE and dl,dr = FAR then dir is ???

  //If dl is CLOSE and dr is FAR then dir is RIGHT 
  dir_right = max(min(min(fs_close(dl),fs_far(dr)),fs_far(dr_rear)),dir_right);
  
  //If dr is CLOSE and dl is FAR the dir is LEFT
  dir_left = max(min(min(fs_close(dr),fs_far(dl)),fs_far(dl_rear)), dir_left);
  
  //Defuzzify and return movement direction
  return f_defuz(dir_left, dir_fwd, dir_right);
}

/*
 *  Defuzzification
 */

float f_defuz(float dir_left, float dir_fwd, float dir_right){ 
  float dir = 0; //Movement direction -1 = strafe left, 0 = forward, 1 = strafe right

  // Simply take membership function with greatest value to define movement direction
  if(dir_right >= dir_left & dir_right >= dir_fwd){ // Move right
    dir = 1;    
  }else if(dir_left >= dir_fwd & dir_left >= dir_right){ // Move left
    dir = -1;
  }else{ //Move fwd
    dir = 0;
  }

  return dir; 
}
