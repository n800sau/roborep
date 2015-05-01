#include "requestParser.h"

requestParser::requestParser(String request)
{
	parseRequest(request);
}

void requestParser::parseRequest(String request)
{
	elm_t elm;
	bool name_stage = true;
	for(int i=0; i<=request.length(); i++) {
		if(i == request.length() || request[i] == ',' ) { // separator
			Serial.print(elm.name);
			Serial.print("=");
			Serial.println(elm.val);
			this->elms.push(elm);
			elm.name = "";
			elm.val = "";
			name_stage = true;
		} else if(request[i] == '=') {
			name_stage = false;
		} else {
			if(name_stage) {
				elm.name += request[i];
			} else {
				elm.val += request[i];
			}
		}
	}
}

String requestParser::operator[](String index)
{
	String rs;
	for(int i=0; i<elms.count(); i++) {
		elm_t elm = elms.get(i);
		if(elm.name == index) {
			rs = elm.val;
		}
	}
	return rs;
}

String requestParser::operator[](const char *index)
{
	return (*this)[String(index)];
}

bool requestParser::success()
{
	return true;
}

