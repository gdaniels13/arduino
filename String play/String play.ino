
// String play.ino

void setup() {
		Serial.begin(9600);
}

int count=0;
void loop() {
		String temp = "hello ";
		Serial.println(temp + getCount(););
}




String getDate(){
	count++;
	return String(count);
}

