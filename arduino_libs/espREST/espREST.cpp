#include "espREST.h"
#include <ArduinoJson.h>
#include <PString.h>

#define MAX_JSON_SIZE 1000

void espREST::set_id(const char *id)
{
	this->id = String(id);
}

void espREST::set_name(const char *name)
{
	this->name = String(name);
}

void espREST::function(const char *name, FTYPE (*f)(String, String&))
{
	if(p_count<VAR_COUNT) {
		p_names[p_count] = String(name);
		p_vars[p_count++] = f;
	}
}

void espREST::variable(const char *name, String *s)
{
	if(s_count<VAR_COUNT) {
		s_names[s_count] = String(name);
		s_vars[s_count++] = s;
	}
}

void espREST::variable(const char *name, int *i)
{
	if(i_count<VAR_COUNT) {
		i_names[i_count] = String(name);
		i_vars[i_count++] = i;
	}
}

void espREST::variable(const char *name, float *f)
{
	if(f_count<VAR_COUNT) {
		f_names[f_count] = String(name);
		f_vars[f_count++] = f;
	}
}

void espREST::begin(ESP8266WebServer &server)
{
	server.onNotFound([this, &server]() {
		this->handle(server);
	});
}

void espREST::handle(ESP8266WebServer &server)
{
	bool error = true;
	DynamicJsonBuffer jsonBuffer, resultBuffer;
	char result_buf[MAX_JSON_SIZE];
	JsonObject& root = jsonBuffer.createObject();
	root["id"] = id.c_str();
	root["name"] = name.c_str();
	String uri = server.uri();
	if(uri.startsWith("/mode/")) {
		int mindex = uri.indexOf("/", 6);
		String pin = uri.substring(6, mindex);
		if(isDigit(pin.charAt(0))) {
			String mode = uri.substring(mindex+1);
			if(mode == "i") {
				pinMode(pin.toInt(), INPUT);
				error = false;
			} else if (mode == "o") {
				pinMode(pin.toInt(), OUTPUT);
				error = false;
			}
		}
	} else if(uri.startsWith("/digital/")) {
		int mindex = uri.indexOf("/", 6);
		String pin = uri.substring(6, mindex);
		if(isDigit(pin.charAt(0))) {
			String val = uri.substring(mindex+1);
			if (val == "") {
				root["result"] = digitalRead(pin.toInt());
				error = false;
			} else if(val == "1") {
				digitalWrite(pin.toInt(), 1);
				error = false;
			} else if (val == "0") {
				digitalWrite(pin.toInt(), 0);
				error = false;
			}
		}
	} else if(uri.startsWith("/analog/")) {
		int mindex = uri.indexOf("/", 6);
		String pin = uri.substring(6, mindex);
		if(isDigit(pin.charAt(0))) {
			String val = uri.substring(mindex+1);
			if (val == "") {
				root["result"] = analogRead(pin.toInt());
				error = false;
			} else if(isDigit(val.charAt(0))) {
				analogWrite(pin.toInt(), val.toInt());
				error = false;
			}
		}
	} else {
		bool found = false;
		int i;
		if(!found) {
			for(i=0; i<s_count; i++) {
				if(s_names[i] == uri.substring(1)) {
					found = true;
					root["result"] = (*s_vars[i]).c_str();
					error = false;
					break;
				}
			}
		}
		if(!found) {
			for(i=0; i<f_count; i++) {
				if(f_names[i] == uri.substring(1)) {
					found = true;
					root["result"] = *f_vars[i];
					error = false;
					break;
				}
			}
		}
		if(!found) {
			for(i=0; i<i_count; i++) {
				if(i_names[i] == uri.substring(1)) {
					found = true;
					root["result"] =  *i_vars[i];
					error = false;
					break;
				}
			}
		}
		if(!found) {
			for(i=0; i<p_count; i++) {
				if(p_names[i] == uri.substring(1)) {
					found = true;
					String result;
					FTYPE ftype = p_vars[i](server.arg("params"), result);
					error = false;
					switch(ftype) {
						case FT_ERROR:
							error = true;
							break;
						case FT_STRING:
							root["result"] = result.c_str();
							break;
						case FT_INT:
							root["result"] = result.toInt();
							break;
						case FT_FLOAT:
							root["result"] = result.toFloat();
							break;
						case FT_JSON:
							{
								result.toCharArray(result_buf, MAX_JSON_SIZE);
		Serial1.print("before:");
		Serial1.println(result_buf);
								root["result"] = resultBuffer.parseObject(result_buf);
							}
							break;
					}
					break;
				}
			}
		}
	}
	if(error) {
		root["error"] = "Wrong request";
	} else if(!root.containsKey("result")) {
		root["result"] = "ok";
	}
	char msgbuf[MAX_JSON_SIZE];
	PString pmsg(msgbuf, MAX_JSON_SIZE);
	root.printTo(pmsg);
	server.send((error) ? 404 : 200, "application/json", msgbuf);
}
