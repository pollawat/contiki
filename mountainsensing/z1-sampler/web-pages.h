#define HTTP_RES "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n"

#define TOP "<html><body>"
#define BOTTOM "</body></html>"
//static const uint16_t TOP_BOTTOM_SIZE = strlen(TOP) + strlen(BOTTOM);

#define CLOCK_FORM "<form action=\"/clock\" method=\"get\">"                \
    "Year<input type=\"number\" name=\"y\" min=\"2014\"><br>"               \
    "Month<input type=\"number\" name=\"mo\" min=\"1\" max=\"12\"><br>"     \
    "Day<input type=\"number\" name=\"d\" min=\"1\" max=\"31\"><br><br>"    \
    "Hour (24H)<input type=\"number\" name=\"h\" min=\"0\" max=\"23\"><br>" \
    "Min<input type=\"number\" name=\"mi\" min=\"0\" max=\"59\"><br>"       \
    "Sec<input type=\"number\" name=\"s\" min=\"0\" max=\"59\"><br>"        \
    "<input type=\"submit\" name=\"submit\" value=\"Submit\">"              \
    "</form>"
//static uint16_t CLOCK_FORM_SIZE = strlen(CLOCK_FORM);

#define INDEX_BODY "<h1>Index</h1>"             \
    "<a href=\"/clock\">Clock Config</a><br>"   \
    "<a href=\"/sample\">Sensors Config</a><br>"\
    "<a href=\"/comms\">Comms Config</a>"
//static uint16_t INDEX_BODY_SIZE = strlen(INDEX_BODY);

#define SENSOR_FORM_1 "<form action=\"/sensub\" method=\"get\">" \
    "Sample Interval (s)<input type=\"number\" name=\"sample\" min=\"1\"value=\""
#define SENSOR_FORM_2 "\"><br>AVR IDs (.sv)<input type=\"text\" name=\"AVR\"pattern=\"([0-9]{1,3}\.)*[0-9]{1,3}\" value=\""
#define SENSOR_FORM_3 "\"><br>Rain?<input type=\"checkbox\" name=\"rain\"value=\"y\""
#define SENSOR_FORM_4 "><br>ADC1?<input type=\"checkbox\" name=\"adc1\"value=\"y\""
#define SENSOR_FORM_5 "><br>ADC2?<input type=\"checkbox\" name=\"adc2\"value=\"y\""
#define SENSOR_FORM_6 "><br><input type=\"submit\" name=\"submit\"value=\"Submit\"></form>"
//static uint16_t SENSOR_FORM_SIZE = strlen(SENSOR_FORM_1)
//    + strlen(SENSOR_FORM_2) + strlen(SENSOR_FORM_3) + strlen(SENSOR_FORM_4)
//    + strlen(SENSOR_FORM_5) + strlen(SENSOR_FORM_6);

#define COMMS_FORM_1 "<form action=\"/comsub\" method=\"get\">" \
    "Comms Interval (s)<input type=\"number\" name=\"interval\" min=\"1\"value=\""
#define COMMS_FORM_2A "\"><br>Gateway IP<input type=\"text\" size=\"4\"name=\"a\" value=\""
#define COMMS_FORM_2B "\"><input type=\"text\" name=\"b\" size=\"4\" value=\""
#define COMMS_FORM_2C "\"><input type=\"text\" name=\"c\" size=\"4\" value=\""
#define COMMS_FORM_2D "\"><input type=\"text\" name=\"d\" size=\"4\" value=\""
#define COMMS_FORM_2E "\"><input type=\"text\" name=\"e\" size=\"4\" value=\""
#define COMMS_FORM_2F "\"><input type=\"text\" name=\"f\" size=\"4\" value=\""
#define COMMS_FORM_2G "\"><input type=\"text\" name=\"g\" size=\"4\" value=\""
#define COMMS_FORM_2H "\"><input type=\"text\" name=\"h\" size=\"4\" value=\""
#define COMMS_FORM_3 "\"><br>Gateway Port<input type=\"text\" name=\"port\"value=\""
#define COMMS_FORM_4 "\"><br><input type=\"submit\" name=\"submit\"value=\"Submit\"></form>"

