void clearLCD ()
{
  //clear the LCD by writing all spaces
  jumpToStart();
  mySerial.write("                ");
  mySerial.write("                ");
  jumpToStart();
}

void jumpToStart ()
{
  //jump to the start of the first line on the LCD
  mySerial.write(254);
  mySerial.write(128);
}

void jumpToLineTwo ()
{
  //jump to the start of the second line on the LCD
  mySerial.write(254);
  mySerial.write(192);
}

void writeLCD (char myText[])
{
  //write text to the LCD
  mySerial.write(myText); 
}
