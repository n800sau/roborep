#include "ADXL345.h"
#include <syslog.h>
#include <math.h>
#include <evhttp.h>
#include <string.h>
#include <unistd.h>
#include <libgen.h>
#include <fcntl.h>
#include <sys/stat.h>

#define ADXL345_I2C_ADDR 0x53

#define Register_PowerControl 0x2D
#define Register_DataFormat 0x31
#define Register_DataX 0x32
#define Register_DataY 0x34
#define Register_DataZ 0x36

#define ErrorCode_1 "Entered range was invalid. Should be 2, 4, 8 or 16g."
#define ErrorCode_1_Num 1

#define ScaleFor2G 0.0039
#define ScaleFor4G 0.0078
#define ScaleFor8G 0.0156
#define ScaleFor16G 0.0312

ADXL345::ADXL345(int port):ReServant("adxl345"),m_Scale(1),raw(),scaled(),xz_degrees(0),yz_degrees(0),port(port)
{
}

void ADXL345::create_servant()
{
	ReServant::create_servant();
	/* initialise ADXL345 */
//	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	setRange(2, true);
//	i2cwire.writeToDevice(Register_DataFormat, 0);
//	i2cwire.writeToDevice(Register_DataFormat, 11);
	if(port > 0) {
		runHttpd("0.0.0.0", port);
	}
}


int ADXL345::setRange(int gNum, bool fullResolution)
{
	uint8_t data;
	syslog(LOG_NOTICE, "Setting range to: %dg", gNum);

	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	// Get current data from this register.
	i2cwire.requestFromDevice(Register_DataFormat, 1, &data);

	// We AND with 0xF4 to clear the bits are going to set.
	// Clearing ----X-XX
	data &= 0xF4;

	// By default (range 2) or FullResolution = true, scale is 2G.
	int m_Scale = ScaleFor2G;
	
	// Set the range bits.
	switch(gNum)
	{
		case 2:
			break;
		case 4:
			data |= 0x01;
			if(!fullResolution) { m_Scale = ScaleFor4G; }
			break;
		case 8:
			data |= 0x02;
			if(!fullResolution) { m_Scale = ScaleFor8G; }
			break;
		case 16:
			data |= 0x03;
			if(!fullResolution) { m_Scale = ScaleFor16G; }
			break;
		default:
			return ErrorCode_1_Num;
	}

	// Set the full resolution bit.
	if(fullResolution)
		data |= 0x08;

	i2cwire.writeToDevice(Register_DataFormat, data);
	return 0;
}

AccelerometerRaw ADXL345::readRawAxis()
{
	int16_t buf[3];
	i2cwire.selectDevice(ADXL345_I2C_ADDR, "ADXL345");
	i2cwire.requestFromDevice(Register_DataX, 6, (uint8_t*)buf);
	AccelerometerRaw raw = AccelerometerRaw();
//	raw.XAxis = (buf[1] << 8) | buf[0];
//	raw.YAxis = (buf[3] << 8) | buf[2];
//	raw.ZAxis = (buf[5] << 8) | buf[4];
	raw.XAxis = buf[0];
	raw.YAxis = buf[1];
	raw.ZAxis = buf[2];
	return raw;
}

AccelerometerScaled ADXL345::readScaledAxis()
{
	AccelerometerRaw raw = readRawAxis();
	AccelerometerScaled scaled = AccelerometerScaled();
	scaled.XAxis = raw.XAxis * m_Scale;
	scaled.YAxis = raw.YAxis * m_Scale;
	scaled.ZAxis = raw.ZAxis * m_Scale;
	return scaled;
}

void ADXL345::enableMeasurements()
{
	syslog(LOG_NOTICE, "Enabling measurements.");
	i2cwire.writeToDevice(Register_PowerControl, 0);
	i2cwire.writeToDevice(Register_PowerControl, 16);
	i2cwire.writeToDevice(Register_PowerControl, 8);
}


float ADXL345::heading(float axis1, float axis2)
{
	float heading = atan2(axis1, axis2);
	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2 * M_PI;
	// Check for wrap due to addition of declination.
	if(heading > 2 * M_PI)
		heading -= 2 * M_PI;
	// Convert radians to degrees for readability.
	return heading * 180 / M_PI;
}

void ADXL345::loop()
{

	raw = readRawAxis();
	scaled = readScaledAxis();
	xz_degrees = heading(scaled.XAxis, scaled.ZAxis);
	yz_degrees = heading(scaled.YAxis, scaled.ZAxis);

	//syslog(LOG_NOTICE, "Raw:%d %4d %4d\n", raw.XAxis, raw.YAxis, raw.ZAxis);
	//syslog(LOG_NOTICE, "Scaled:%g %g %g\n", scaled.XAxis, scaled.YAxis, scaled.ZAxis);
	//syslog(LOG_NOTICE, "XZ:%g YZ:%g\n", xz_degrees, yz_degrees);

	json_t *js = json_object();
	json_object_set_new(js, "mypath", json_string(mypath()));
	json_object_set_new(js, "s_timestamp", json_string(s_timestamp()));
	json_object_set_new(js, "rawXAxis", json_integer(raw.XAxis));
	json_object_set_new(js, "rawYAxis", json_integer(raw.YAxis));
	json_object_set_new(js, "rawZAxis", json_integer(raw.ZAxis));
	json_object_set_new(js, "scaledXAxis", json_real(scaled.XAxis));
	json_object_set_new(js, "scaledYAxis", json_real(scaled.YAxis));
	json_object_set_new(js, "scaledZAxis", json_real(scaled.ZAxis));
	json_object_set_new(js, "xz_degrees", json_real(xz_degrees));
	json_object_set_new(js, "yz_degrees", json_real(yz_degrees));
	char *jstr = json_dumps(js, JSON_INDENT(4));
	if(jstr) {
		syslog(LOG_DEBUG, "%s\n", jstr);
		redisAsyncCommand(aredis, NULL, NULL, "RPUSH %s.js.obj %s", myid(), jstr);
		redisAsyncCommand(aredis, NULL, NULL, "LTRIM %s.js.obj %d -1", myid(), -REDIS_LIST_SIZE-1);
		free(jstr);
	} else {
		syslog(LOG_ERR, "Can not encode JSON\n");
	}
	json_decref(js);

	redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.raw_x %d", myid(), raw.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.raw_y %d", myid(), raw.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.i.raw_z %d", myid(), raw.ZAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_x %g", myid(), scaled.XAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_y %g", myid(), scaled.YAxis);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.scaled_z %g", myid(), scaled.ZAxis);


	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.xz %g", myid(), xz_degrees);
	redisAsyncCommand(aredis, NULL, NULL, "SET %s.r.yz %g", myid(), yz_degrees);

	ReServant::loop();
}

int fsize(FILE *fp){
    int prev=ftell(fp);
    fseek(fp, 0L, SEEK_END);
    int sz=ftell(fp);
    fseek(fp,prev,SEEK_SET); //go back to where we were
    return sz;
}

void ADXL345::http_request(struct evhttp_request *req)
{
//	ReServant::http_request(req);
	const char *cmdtype;
	struct evkeyvalq *headers;
	struct evkeyval *header;
	struct evbuffer *buf;

	switch (evhttp_request_get_command(req)) {
		case EVHTTP_REQ_GET: cmdtype = "GET"; break;
		case EVHTTP_REQ_POST: cmdtype = "POST"; break;
		case EVHTTP_REQ_HEAD: cmdtype = "HEAD"; break;
		case EVHTTP_REQ_PUT: cmdtype = "PUT"; break;
		case EVHTTP_REQ_DELETE: cmdtype = "DELETE"; break;
		case EVHTTP_REQ_OPTIONS: cmdtype = "OPTIONS"; break;
		case EVHTTP_REQ_TRACE: cmdtype = "TRACE"; break;
		case EVHTTP_REQ_CONNECT: cmdtype = "CONNECT"; break;
		case EVHTTP_REQ_PATCH: cmdtype = "PATCH"; break;
		default: cmdtype = "unknown"; break;
	}

	syslog(LOG_NOTICE, "Received a %s request for %s", cmdtype, evhttp_request_get_uri(req));

	headers = evhttp_request_get_input_headers(req);
	for (header = headers->tqh_first; header;
	    header = header->next.tqe_next) {
		syslog(LOG_NOTICE, "  %s: %s\n", header->key, header->value);
	}

	buf = evhttp_request_get_input_buffer(req);
	syslog(LOG_NOTICE, "Input data: <<<");
	while (evbuffer_get_length(buf)) {
		int n;
		char cbuf[128];
		n = evbuffer_remove(buf, cbuf, sizeof(buf)-1);
		if (n > 0)
			syslog(LOG_NOTICE, "%s", cbuf);
	}
	syslog(LOG_NOTICE, ">>>");

	buf = evbuffer_new();
	if(strcmp(req->uri, "/") == 0) {
		char index_path[400];
		strcpy(index_path, mypath());
		strcat(index_path, "/index.html");
		int fd = open(index_path, O_RDONLY);
		if(fd >= 0) {
			struct stat st;
			fstat(fd, &st);
			evbuffer_add_file(buf, fd, 0, st.st_size);
//			evbuffer_read(buf, fd, -1);
			syslog(LOG_NOTICE, "file size=%d, bufsize=%d", st.st_size, evbuffer_get_length(buf));
			evhttp_send_reply(req, 200, "OK", buf);
			close(fd);
		} else {
			syslog(LOG_WARNING, "Error opening %s", index_path);
			evhttp_send_reply(req, 500, "Internal Error", buf);
		}
	} else {
		json_t *js = json_object();
		json_object_set_new(js, "mypath()", json_string(mypath()));
		json_object_set_new(js, "s_timestamp", json_string(s_timestamp()));
		json_object_set_new(js, "rawXAxis", json_integer(raw.XAxis));
		json_object_set_new(js, "rawYAxis", json_integer(raw.YAxis));
		json_object_set_new(js, "rawZAxis", json_integer(raw.ZAxis));
		json_object_set_new(js, "scaledXAxis", json_real(scaled.XAxis));
		json_object_set_new(js, "scaledYAxis", json_real(scaled.YAxis));
		json_object_set_new(js, "scaledZAxis", json_real(scaled.ZAxis));
		json_object_set_new(js, "xz_degrees", json_real(xz_degrees));
		json_object_set_new(js, "yz_degrees", json_real(yz_degrees));
		char *jstr = json_dumps(js, JSON_INDENT(4));
		if(jstr) {
			syslog(LOG_DEBUG, "%s\n", jstr);
			evbuffer_add(buf, jstr, strlen(jstr));
			headers = evhttp_request_get_output_headers(req);
			evhttp_add_header(headers, "Content-Type", "application/json");
			evhttp_send_reply(req, 200, "OK", buf);
			free(jstr);
		} else {
			syslog(LOG_ERR, "Can not encode JSON\n");
			evhttp_send_reply(req, 500, "Internal Error", buf);
		}
		json_decref(js);
	}
	evbuffer_free(buf);
	syslog(LOG_NOTICE, "URI:%s", req->uri);

}


