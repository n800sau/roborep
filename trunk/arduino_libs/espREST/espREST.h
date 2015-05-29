#ifndef __ESPREST_H

#define __ESPREST_H

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define VAR_COUNT 10

enum FTYPE {FT_NONE, FT_ERROR, FT_JSON, FT_INT, FT_FLOAT, FT_STRING};

class espREST {
	protected:
		String id;
		String name;

		String *s_vars[VAR_COUNT];
		String s_names[VAR_COUNT];
		int s_count;

		int *i_vars[VAR_COUNT];
		String i_names[VAR_COUNT];
		int i_count;

		float *f_vars[VAR_COUNT];
		String f_names[VAR_COUNT];
		int f_count;

		FTYPE (*p_vars[VAR_COUNT])(String, String&);
		String p_names[VAR_COUNT];
		int p_count;
	public:
		espREST():s_count(0),i_count(0),f_count(0),p_count(0){}
		void set_id(const char *id);
		void set_name(const char *name);
		void function(const char *name, FTYPE (*f)(String param, String &result));
		void variable(const char *name, String *s);
		void variable(const char *name, int *i);
		void variable(const char *name, float *f);
		void begin(ESP8266WebServer &server);
		void handle(ESP8266WebServer &server);
};


#endif //__ESPREST
