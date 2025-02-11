int boolLid=0;
int LidPinOpen = 27 ;
int LidPinClose = 26 ;
void lidOpen(){
    if(boolLid==0)
    analogWrite(LidPinOpen,255);
    delay(2000);
    analogWrite(LidPinOpen, 0);
    boolLid=1;
}

void lidClose(){
    if(boolLid==1){
        analogWrite(LidPinClose,255);
        delay(2000);
        analogWrite(LidPinClose, 0);
        boolLid=0;

    }
}

void ClawClose(){

}

void ClawOpen(){

}

void setup() {
  Serial.begin(9600);
  Serial.println("run");
}

void loop()
{
  if (Serial.available() > 0)
  {
        String str = Serial.readStringUntil('\n');  // Read data until newline
        str.trim();  // Remove extra spaces or newlines
        char* command = str.c_str(); // convert String to char*
        int arr[2];

        // converting char* to int array
        char *p = strtok(command, ",");
        size_t index = 0;

        while (p != nullptr && index<2)
        {
          arr[index++] = atoi(p);
          p = strtok(NULL, ",");
        }


        if (arr[0] == 1)

          {
            // write for when object is detected
            delay(2000);
            Serial.println("run");
          }

        else if (arr[0] == 2)
          {
            // write for when nothing detected
            delay(2000);
            Serial.println("run");
          }
  }
}