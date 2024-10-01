#include <RIC3DMODEM.h>
#include <RIC3D.h>
#include <TimerOne.h>
#include <floatToString.h>

#define SerialMon Serial // Monitor serial para depuración
#define SerialAT Serial3 // Comunicación con el módem 4G

#define BASE_SENSE_TIME 1000
#define SEND_REPORT_TIME BASE_SENSE_TIME * 15 * 1000
#define PORT_CORRECTION_FACTOR 40
// this initializes the device setting the corresponding inputs and outputs
RIC3D device;

/**************************************
 * COMMS
 *************************************/
// Configuración del módem (ajustar según el hardware y proveedor)
const char apn[] = "grupotesacom.claro.com.ar"; // APN de la red móvil
const char gprsUser[] = "";                     // Usuario GPRS (si es necesario)
const char gprsPass[] = "";                     // Contraseña GPRS (si es necesario)

const char client_id[] = "vBSeBu64ji8qaHCZMLZo"; // Aca se debe poner el TOKEN del device de tdata
const char mqtt_host[] = "10.25.1.152";
const char mqtt_port[] = "4090";

// Module baud rate
uint32_t rate = 115200;

// Select SIM Card (0 = right, 1 = left)
bool sim_selected = 1;

void modemSetup()
{

  SerialMon.println(F("modemSetup()"));
  pinMode(SIM_SELECT, OUTPUT);
  SerialAT.begin(rate);
  ModemBegin(&SerialAT, &SerialMon);
  digitalWrite(SIM_SELECT, sim_selected);

#ifdef DUMP_AT_COMMANDS
  ModemBegin(&SerialDebugger, &SerialMon);
#else
  ModemBegin(&SerialAT, &SerialMon);
#endif

  ModemTurnOff();
  ModemTurnOn();
}

int modemInit()
{
  ModemTurnOff();
  ModemTurnOn();
  int result;
  ATtest();
  SerialMon.print(F("ATtest: "));
  if (result = ATtest())
    return result;
  SerialMon.println(F("ok"));
  SerialMon.print(F("CreatePDPContext: "));
  if (result = CreatePDPContext(apn, gprsUser, gprsPass))
    return result;
  SerialMon.println(F("ok"));
  SerialMon.print(F("ActivatePDPContext: "));
  if (result = ActivatePDPContext())
    return result;
  SerialMon.println(F("ok"));
  SerialMon.print(F("ConnectMQTTClient: "));
  if (result = ConnectMQTTClient(client_id, mqtt_host, mqtt_port))
    return result;
  SerialMon.println(F("ok"));
  return result;
}

/**************************************
 * UTILS
 **************************************/

char read_analog_input(uint8_t port)
{
  // float min_sensor_value =
  int sensor_value = analogRead(port);
  // Serial.print("Sensor value: ");
  Serial.print(sensor_value);
  Serial.print("\n");

  if (sensor_value == 0)
  {
    return -1;
  }
  return sensor_value;
}

/*********************************
 * Filters
 **********************************/
class Filter
{
public:
  virtual float apply(float value) = 0;
};

class SMA : public Filter
{
public:
  SMA(uint8_t period) : period(period)
  {
    sma_values = new float[period];
    for (int i = 0; i < period; i++)
    {
      sma_values[i] = 0;
    }
  }
  SMA() : SMA(5) {}
  ~SMA()
  {
    delete[] sma_values;
  }
  float apply(float new_value)
  {
    sma_values[sma_index++] = new_value;
    sma_index %= period;
    float sum = 0;
    for (int i = 0; i < period; i++)
    {
      sum += sma_values[i];
    }
    Serial.print(sum / period);
    Serial.print(", ");
    return sum / period;
  }

  float alt_update(float new_value)
  {
    sma_sum -= sma_values[sma_index];
    sma_sum += new_value;
    sma_values[sma_index] = new_value;
    sma_index = ++sma_index % period;
    return (sma_sum) / period;
  }

private:
  uint8_t period;
  float *sma_values;
  uint8_t sma_index = 0;
  float sma_sum = 0;
};

class EMA : public Filter
{
public:
  EMA(float _alpha)
  {
    alpha = _alpha;
  }
  EMA() : EMA(0.1) {}
  float apply(float new_value)
  {
    ema = alpha * new_value + (1 - alpha) * ema;
    return ema;
  }

private:
  float alpha;
  float ema = 0.0;
};

/**************************************
 * SENSORS
 **************************************/

class AnalogSensor
{
private:
  String name;
  uint8_t port;
  float correction;
  float *mem;
  uint8_t mem_size;
  uint8_t mem_index = 0;
  Filter *filter;
  float last_raw_value = 0;
  float last_value = 0;
  float last_computed_value = 0;
  float min_value = 0;
  float max_value = 0;
  bool use_filter_for_report;
  float unit_zero;
  float unit_factor;
  bool is_dead = false;

public:
  AnalogSensor(
      String name,
      uint8_t port,
      float unit_zero,
      float unit_factor,
      Filter *filter,
      float correction = 0,
      bool use_filter_for_report = true) : name(name),
                                           port(port),
                                           unit_zero(unit_zero),
                                           unit_factor(unit_factor),
                                           filter(filter),
                                           correction(correction),
                                           use_filter_for_report(use_filter_for_report)
  {
    this->mem_size = (SEND_REPORT_TIME / BASE_SENSE_TIME) + 1;
    this->mem = new float[mem_size];
  }

  ~AnalogSensor()
  {
    delete[] mem;
  }

  float read()
  {
    float sensor_value = analogRead(port) / PORT_CORRECTION_FACTOR + correction;
    Serial.print(sensor_value);
    Serial.print(", ");
    if (sensor_value < 4 || sensor_value > 20)
    {
      if (!is_dead)
      {
        is_dead = true;
        return -1;
      }
    }
    if (filter != nullptr)
    {
      last_value = filter->apply(sensor_value);
      if (use_filter_for_report)
      {
        sensor_value = last_value;
      }
    }
    Serial.print(sensor_value);
    Serial.print(", ");
    if (mem_index == mem_size)
    {
      Serial.println("Memory overload on sensor ");
      Serial.println(name);
      return -1;
    }
    last_computed_value = sensor_value * unit_factor + unit_zero;
    mem[mem_index++] = last_computed_value;
    return last_computed_value;
  }

  float get_last_raw_value() { return last_raw_value; }

  float get_last_value() { return last_value; }

  float get_last_computed_value() { return last_computed_value; }

  String get_name() { return name; }

  bool get_is_dead()
  {
    return is_dead;
  }

  void send_report()
  {
    float sum = 0;
    for (int i = 0; i < mem_size; i++)
    {
      sum += mem[i];
    }
    float avg = sum / mem_size;
    mem_index = 0;
    // COMMUNICATION
    char buff[10] = {0};
    floatToString(min_value, buff, sizeof(buff), 3);
    PublishData("min", buff);
    floatToString(max_value, buff, sizeof(buff), 3);
    PublishData("max", buff);
    floatToString(avg, buff, sizeof(buff), 3);
    PublishData("avg", buff);
  }
};

/**************************************
 * Main
 **************************************/

float last_t = 0;

void setup()
{
  // device.begin();
  Serial.begin(115200);
  // Set internal reference of the adc to 2,56V
  analogReference(INTERNAL2V56);

  modemSetup();
  int result;
  while (result = modemInit())
  {
    SerialMon.print(F("falló! -> "));
    SerialMon.println(result);
    delay(10000);
  }

  last_t = millis();
  Serial.println("Device is ready!");
}

SMA flowmeter_sma(20);

AnalogSensor flowmeter(
    "Caudalimetro",
    AI0,
    4 / 16,
    50 / 16,
    &flowmeter_sma,
    -0.3,
    true);

void loop()
{
  float dt = millis() - last_t;
  if (dt >= SEND_REPORT_TIME)
  {
    flowmeter.send_report();
    last_t = millis();
  }
  float val = flowmeter.read();
  Serial.print(val);

  Serial.print("\n");
  delay(BASE_SENSE_TIME);
}
