#include "limited_queue.h"

typedef struct _elm_t {
	String name;
	String val;
} elm_t;

class requestParser {

	protected:

		LimitedQueue<elm_t, 10> elms;
		void parseRequest(String request);

	public:
		requestParser(String request);
		String operator[](String index);
		String operator[](const char *index);
		bool success();
};
