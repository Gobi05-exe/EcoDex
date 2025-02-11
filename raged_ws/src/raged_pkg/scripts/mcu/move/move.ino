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
        
        // convert String to char*
        char commandCopy[str.length() + 1];  // Create a writable character array
        strcpy(commandCopy, str.c_str());    // Copy the string into it
        char *p = strtok(commandCopy, ",");
        
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
            delay(2000);
            Serial.println("run");
          }

        else if (arr[0] == 2)
          {
            delay(2000);
            Serial.println("run");
          }
        
        else if (arr[0] == 3)
          {
            delay(2000);
            Serial.println("run");
          }
        
        else if (arr[0] == 4)
          {
            delay(2000);
            Serial.println("run");
          }
  }
}
