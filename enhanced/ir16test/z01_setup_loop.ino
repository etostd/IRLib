int RECV_PIN = 11;

IRrecv My_Receiver(RECV_PIN);

IRdecode My_Decoder;
unsigned int Buffer[RAWBUF];

void setup()
{
  Serial.begin(9600);
  delay(2000);while(!Serial);//delay for Leonardo
  My_Receiver.enableIRIn(); // Start the receiver
  My_Decoder.UseExtnBuf(Buffer);
}

void loop() {
  if (My_Receiver.GetResults(&My_Decoder)) {
    //Restart the receiver so it can be capturing another code
    //while we are working on decoding this one.
    My_Decoder.decode();
    My_Decoder.DumpResults();
    My_Receiver.resume(); 
  }
}

