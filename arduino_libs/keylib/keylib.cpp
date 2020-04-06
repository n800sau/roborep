#include <keylib.h>

readKey::readKey():
	last_key(0), last_change_millis(0)
{
}

void readKey::begin(int ocount, int icount, const uint8_t *poline,  const uint8_t *piline, unsigned long debounce_millis)
{
	this->poline = poline;
	this->piline = piline;
	this->icount = icount;
	this->ocount = ocount;
	this->debounce_millis = debounce_millis;
	for(byte i=0; i<ocount; i++) {
		pinMode(poline[i], OUTPUT);
		digitalWrite(poline[i], HIGH);
	}

	for(byte i=0; i<icount; i++) {
		pinMode(piline[i], INPUT_PULLUP);
	}
}

byte readKey::read_key()
{
	byte rs = 0;
	for(byte i=0; i<ocount && rs == 0; i++) {
		digitalWrite(poline[i], LOW);
		for(byte j=0; j<icount; j++) {
			if(digitalRead(piline[j]) == LOW) {
				rs = (i+1)<<4 | (j+1);
				break;
			}
		}
		digitalWrite(poline[i], HIGH);
	}
	return rs;
}

byte readKey::read_key_debounce() {
	byte rs = read_key();
	if(last_key != rs) {
		unsigned long m = millis();
		if(rs == 0 && debounce_millis + last_change_millis < m) {
			rs = last_key;
			last_change_millis = m;
		} else {
			last_key = rs;
		}
	}
	return rs;
}
