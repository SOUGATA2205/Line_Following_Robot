char H = HIGH;
char L = LOW;

#define L1  5 
#define L2  7
#define R1  2
#define R2  4
#define MR  3
#define ML  6

int i, r[4], l[4], pid_val;
int err,yStage = 0;
int readL cumm_err, prev_err = 0;
int repla+ength = 0, pathLength = 1, checkLength = 0;
int path[40] = {};
int s[7] = {};
int mr = 170;
int ml = 210;
int mrt = 0.4*mr;
int mlt = 0.4*ml;
float mr_fac = ((float) mr)/255;
float ml_fac = ((float) ml)/255;
int mr_val, ml_val;
int for_delay = 100;
int turn_delay = 200;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(MR, OUTPUT);
  pinMode(ML, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  setColor(1, 1, 1);

   if(replayStage == 0){
    readSensors();
    if((l[3] == 1) && (l[2] == 1) && (l[1] == 1) && (l[0] == 1) && (r[0] == 1) && (r[1] == 1) && (r[2] == 1) && (r[3] == 1)){
     check_path();
    }
    else if((l[3] == 0) && (l[2] == 0) && (l[1] == 0) && (l[0] == 0)){
      check_stop();
    }
    else if((l[3] == 1) && (r[0] == 0) && (r[1] == 0) && (r[2] == 0) && (r[3] == 0)){
      check_right();
    }
    else{
      linefollow();
    }
   }
  /* else{
     replay();
   }*/
}

void readSensors(){
  r[3] = digitalRead(A0);
  r[2] = digitalRead(A1) ;
  r[1] = analogRead(A2) < 120 ? 0 : 1;
  r[0] = digitalRead(A3);
  l[0] = digitalRead(A4);
  l[1] = digitalRead(A5);
  l[2] = analogRead(A6) < 500 ? 0 : 1;
  l[3] = analogRead(A7) < 500 ? 0 : 1;
}
void readErrors(){
   readSensors();

  s[0] = (1-r[3]);
  s[1] = (1-r[2]);
  s[2] = (1-r[1]);
  s[3] = (1-r[0]);
  s[4] = (1-l[0]);
  s[5] = (1-l[1]);
  s[6] = (1-l[2]);
  s[7] = (1-l[3]);

  for(int i=7; i>=0; i--){
    Serial.print(s[i]);
  }
  Serial.print("  ");
}

int cal_errors(){
  readErrors();
  int error = 0;
  for(int i=0; i < 8; i++){
    error+= ((2*i)-7)*s[i];
  }
  return error;
}

void follow_path(){
  float kp = 7, ki = 0, kd = 6.5;
  int raw_err = cal_errors();
  
  if(l[3] == 0){
    err = 30 - raw_err;
  }
  else if(r[3] == 0){
    err = -30 - raw_err;
  }
  else{
    err = raw_err;
  }

  cumm_err+= err;
  pid_val = ((kp * err) + (ki * cumm_err) + (kd * (err - prev_err)));

  float pid_val_min = -min((mr-70)/mr_fac, (ml-70)/ml_fac);
  float pid_val_max = min((255-mr)/mr_fac, (255-ml)/ml_fac);

  pid_val = min(pid_val_max, max(pid_val_min, pid_val));
  mr_val = mr + mr_fac*pid_val;
  ml_val = ml - ml_fac*pid_val;

  Serial.print(",  error= ");
  Serial.print(err);
  Serial.print(",  pid_value= ");
  Serial.print(pid_val);
  Serial.print(",  motor_speed= ");
  Serial.print(ml_val);
  Serial.print(" ");
  Serial.println(mr_val);

  analogWrite(MR, mr_val);
  analogWrite(ML, ml_val);
  digitalWrite(L1, H);
  digitalWrite(L2, L);
  digitalWrite(R1, H);
  digitalWrite(R2, L);

  prev_err = err;
  delay(10);
}

void STOP(){
   analogWrite(MR, 0);
   analogWrite(ML, 0);
   digitalWrite(L1, L);
   digitalWrite(L2, L);
   digitalWrite(R1, L);
   digitalWrite(R2, L);
   delay(200);
}

void forward(){
  analogWrite(MR, mr);
   analogWrite(ML, ml);
   digitalWrite(L1, H);
   digitalWrite(L2, L);
   digitalWrite(R1, H);
   digitalWrite(R2, L);
   delay(for_delay);
}

void check_path(){
  forward();
  delay(300);
  STOP();

  readSensors();
  if((l[3] == 1) && (l[2] == 1) && (l[1] == 1) && (l[0] == 1) && (r[0] == 1) && (r[1] == 1) && (r[2] == 1) && (r[3] == 1)){
    readSensors();
    while((l[0] == 1) && (r[0] == 1)){
     analogWrite(MR, mrt);
     analogWrite(ML, mlt);
     digitalWrite(L1, L);
     digitalWrite(L2, H);
     digitalWrite(R1, L);
     digitalWrite(R2, H);
     readSensors();
   }
    STOP();
    readSensors();
    if((l[3] == 1) && (r[3] == 1))
    {
      back();  
    }
    else{
      readSensors();
    }
  }
  else{
    follow_path();
  }
}

void setColor(int rd, int gn, int bl){
  digitalWrite(10, rd);
  digitalWrite(11, gn);
  digitalWrite(12, bl);
}

void right(){
  forward();
  STOP();
  Serial.println("Right turn");
  
  readSensors();
  while((l[0] == 0) || (r[0] == 0)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, H);
    digitalWrite(L2, L);
    digitalWrite(R1, L);
    digitalWrite(R2, H);
    readSensors();
  }
  while((l[0] == 1) || (r[0] == 1)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, H);
    digitalWrite(L2, L);
    digitalWrite(R1, L);
    digitalWrite(R2, H);
    readSensors();
  }
  STOP();
  
  if(replayStage == 0){
    path[pathLength] = 'R';
    pathLength++;
    if(path[pathLength-2] == 'B'){
      shortPath();  
    }
  }
  readSensors();
}

void left(){
  forward();
  STOP();
  Serial.println("Right turn");
  
  readSensors();
  while((l[0] == 0) || (r[0] == 0)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, L);
    digitalWrite(L2, H);
    digitalWrite(R1, H);
    digitalWrite(R2, L);
    readSensors();
  }
  while((l[0] == 1) || (r[0] == 1)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, L);
    digitalWrite(L2, H);
    digitalWrite(R1, H);
    digitalWrite(R2, L);
    readSensors();
  }
  STOP();
  
  if(replayStage == 0){
    path[pathLength] = 'L';
    pathLength++;
    Serial.print("L");
    if(path[pathLength-2] == 'B'){
      shortPath();
    }
  }
  readSensors();
}

void back(){
  STOP();
  Serial.println("Back");

  readSensors();
  while((l[0] == 0) || (r[0] == 0)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, H);
    digitalWrite(L2, L);
    digitalWrite(R1, H);
    digitalWrite(R2, L);
    readSensors();
   }
   delay(100);
   STOP();

  readSensors();
  while((l[0] == 1) && (r[0] == 1)){
    analogWrite(MR, mrt);
    analogWrite(ML, mlt);
    digitalWrite(L1, H);
    digitalWrite(L2, L);
    digitalWrite(R1, L);
    digitalWrite(R2, H);
    readSensors();
  }
  STOP();
   
  if(replayStage == 0){
    path[pathLength]='B';
    pathLength++;
  }
   readSensors();
}

void straight(){
  forward();
  
  if(replayStage == 0){
    path[pathLength] = 'S';
    pathLength++;
    if(path[pathLength-2] == 'B'){
      shortPath();
    }
  }
  readSensors();
}

void check_right(){
  forward();
   STOP();
  readSensors();
  if((l[3] == 1) && (l[2] == 1) && (l[1] == 1) && (l[0] == 1) && (r[0] == 1) && (r[1] == 1) && (r[2] == 1) && (r[3] == 1)){
    right();
  }
  else{
    straight();
  }
}

void check_stop(){
  forward();
  STOP();
  readSensors();

  if((l[3] == 0) && (l[2] == 0) && (l[1] == 0) && (l[0] == 0) && (r[0] == 0) && (r[1] == 0) && (r[2] == 0) && (r[3] == 0)){
   setColor(0, 1, 1);
   delay(500);
   setColor(1, 0, 1);
   delay(500);
   setColor(1, 1, 0);
   delay(500);
   setColor(0, 0, 0);
   delay(500);
   setColor(1, 1, 1);
   delay(1000);

   forward();
   delay(100);
   analogWrite(MR, mrt);
   analogWrite(ML, mlt);
   digitalWrite(L1, H);
   digitalWrite(L2, L);
   digitalWrite(R1, L);
   digitalWrite(R2, H);
   delay(turn_delay);

   readSensors();
   while((l[0] == 1) && (r[0] == 1)){
     analogWrite(MR, mrt);
     analogWrite(ML, mlt);
     digitalWrite(L1, H);
     digitalWrite(L2, L);
     digitalWrite(R1, L);
     digitalWrite(R2, H);
     readSensors();
   }
   STOP();
   replayStage++;
   pathLength-=1;
   follow_path();
  }
  else{
    left();
  }
}

void check_end(){
  readSensors();
  while((l[0] == 1) && (r[0] == 1)){
   analogWrite(MR, mrt);
   analogWrite(ML, mlt);
   digitalWrite(L1, L);
   digitalWrite(L2, H);
   digitalWrite(R1, L);
   digitalWrite(R2, H);
   readSensors();
  }
  STOP();
  readSensors();
  if((l[3] == 1) && (r[3] == 1) && readLength == 0){
    end_motion();  
  }
  else{
    readSensors();
  }
}

void end_motion(){
   analogWrite(MR, 0);
   analogWrite(ML, 0);
   digitalWrite(L1, L);
   digitalWrite(L2, L);
   digitalWrite(R1, L);
   digitalWrite(R2, L);
   setColor(0, 1, 0);
   delay(500);
   setColor(1, 0, 0);
   delay(500);
   setColor(0, 0, 1);
   delay(500);
   setColor(0, 0, 0);
   delay(500);
   setColor(1, 1, 1);
   delay(1000);
   end_motion();
}

void shortPath(){
  if(path[pathLength-3] == 'L' && path[pathLength-1] == 'R'){
    pathLength-=3;
    path[pathLength] = 'B';
  }
  if(path[pathLength-3] == 'L' && path[pathLength-1] == 'S'){
    pathLength-=3;
    path[pathLength] = 'R';
  }
  if(path[pathLength-3] == 'L' && path[pathLength-1] == 'L'){
    pathLength-=3;
    path[pathLength] = 'S';
  }
  if(path[pathLength-3] == 'S' && path[pathLength-1] == 'L'){
    pathLength-=3;
    path[pathLength] = 'R';
  }
  if(path[pathLength-3] == 'S' && path[pathLength-1] == 'S'){
    pathLength-=3;
    path[pathLength] = 'B';
  }
  if(path[pathLength-3] == 'S' && path[pathLength-1] == 'R'){
    pathLength-=3;
    path[pathLength] = 'L';
  }
  if(path[pathLength-3] == 'R' && path[pathLength-1] == 'L'){
    pathLength-=3;
    path[pathLength] = 'B';
  }
  if(path[pathLength-3] == 'R' && path[pathLength-1] == 'S'){
    pathLength-=3;
    path[pathLength] = 'L';
  }
  if(path[pathLength-3] == 'R' && path[pathLength-1] == 'R'){
    pathLength-=3;
    path[pathLength] = 'S';
  }
  pathLength++;
}

void replay(){
  readLength = (pathLength - checkLength);
  readSensors();
   if((l[3] == 0) && (l[2] == 0) && (l[1] == 0) && (l[0] == 0) && (r[0] == 0) && (r[1] == 0) && (r[2] == 0) && (r[3] == 0)){
    check_end();
    }
  else if((l[3] == 0) && (l[2] == 0) && (l[1] == 0) && (l[0] == 0) && (r[0] == 0) && (r[1] == 0) && (r[2] == 0) && (r[3] == 0)){
    turns();
  }
  else if((l[3] == 0) && (l[2] == 0) && (l[1] == 0) && (l[0] == 0) && (r[0] == 0) && (r[3] == 1)){
    turns(); 
  }
  else if((l[3] == 1) && (l[0] == 0) && (r[0] == 0) && (r[1] == 0) && (r[2] == 0) && (r[3] == 0)){
    turns();
  }
  else{
    follow_path();
  }
  replay();
}

void turns(){
    if(path[readLength] == 'L'){
      right();
    }
    else if(path[readLength] == 'R'){
      left();
    }
    else if(path[readLength] == 'S'){
      follow_path();
    }
    else{
      readSensors();
    }
    checkLength++;
}
