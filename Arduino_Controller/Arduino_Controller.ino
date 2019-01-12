#include <Wire.h>

// Interrupt
#define IN_TERRUPT 7

// Ports values
#define LED 3
#define SENSOR 0
#define MAX_READ 1023.0
#define MAX_WRITE 255.0
#define VCC 5.0
#define IN_I 0

// Identification values BLACK WIRE
#define ID 0
#define OWN_ADD 7
#define M -0.677
#define B 4.95
#define M_TAU 0.003853
#define B_TAU 0.01973

// Controller values BLACK WIRE
#define H 0.01
#define Kp 1
#define Ki 2


//// Identification values WHITE WIRE
//#define ID 1
//#define OWN_ADD 8
//#define M -0.669
//#define B 4.95
//#define M_TAU -0.006505
//#define B_TAU 0.03229
//
//// Controller values
//#define H 0.01
//#define Kp 2
//#define Ki 0.5

// Circuit values
#define R1 10000
#define C 0.000001

// Some numbers (These may already be on Arduino).
#define MAX_D       255
#define TOLERANCE   0.5
#define RHO         0.07
#define Q           0.1
#define C_O         1

// Lower Bound
#define L_Bound_H 100
#define L_Bound_L 20

//I2C Process
#define I2C_CAL 1
#define I2C_CON 2
#define I2C_RASP 4
#define RASP_CONST 2

class Consensus {
  private:
    float y[2] = {0, 0};
    float d[2] = {0, 0};

    float L;
    float o;
    float d_av[2] = {0, 0};
    float k[2];

    bool is_feasible(float di[]);
    float compute_cost(float di[]);
    float linear_boundary(float di[], float cost_best);
    float lower_boundary(float di[], float cost_best);
    float upper_boundary(float di[], float cost_best);
    float linear_and_lower_boundary(float di[], float cost_best);
    float linear_and_upper_boundary(float di[], float cost_best);

  public:
    Consensus() = delete;
    Consensus(float L_, float o_, float k_[]);
    ~Consensus() = default;

    void update_average(float dj[]);
    void primal_solver();
    void update_L(float L_new);
    void update_o(float o_new);
    void update_k(float k_new[]);
    void get_average(float av[]);
    void get_d(float di[]);
    void reset_d_dav_y();
    float get_o();
};

class Trans_data
{
  private:
    int id = ID; //1 bit max
    byte cod = 255; //
    int info[2] = {255, 255}; //8 bit max
    byte tran[4];
  public:
    void put_cod(int mode);
    void int_in_pos(int position, int out_info);
    byte envia();
    void recebe(byte *c, byte mode);
    int get_cod();
    int get_info(int position);
};


///////////////////////////////////////////////////////////////////////////////
//void Trans_data::put_cod(int mode) - Saves the type of message to be sent
///////////////////////////////////////////////////////////////////////////////
void Trans_data::put_cod(int mode)
{
  cod = mode;
}


///////////////////////////////////////////////////////////////////////////////
//void Trans_data::int_in_pos(int position, int out_info) - Saves the information
//to be sent in a position of the message
///////////////////////////////////////////////////////////////////////////////
void Trans_data::int_in_pos(int position, int out_info)
{
  if (cod < I2C_RASP)
  {
    info[position] = out_info;
  }
  if (out_info > 255)
    tran[position] = 255;
  else if (out_info < 0)
    tran[position] = 0;
  else if (cod == I2C_RASP && position == 0)
  {
    if (out_info == RASP_CONST)
    {
      tran[position] = id;
    }
    else
      tran[position] = 0b00000100 + (out_info << 1) + id;
  }
  else
    tran[position] = lowByte(out_info);
}


 
///////////////////////////////////////////////////////////////////////////////
//byte Trans_data::envia() - Sends the message saved in the object. Returns
//error information
///////////////////////////////////////////////////////////////////////////////
byte Trans_data::envia()
{
  Wire.beginTransmission(0);
  Wire.write(tran[0]);
  if (cod > I2C_CAL)
  {
    Wire.write(tran[1]);
    if (cod > I2C_CON)
    {
      Wire.write(tran[2]);
      Wire.write(tran[3]);
    }
  }
  return Wire.endTransmission(1);
}


///////////////////////////////////////////////////////////////////////////////
//void Trans_data::recebe(byte *c, byte mode) - Saves the two first bytes
//of a received message
///////////////////////////////////////////////////////////////////////////////
void Trans_data::recebe(byte *c, byte mode)
{
  cod = mode;
  info[0] = c[0];
  if (mode == I2C_CON)
  {
    info[1] = c[1];
  }
}


///////////////////////////////////////////////////////////////////////////////
//int Trans_data::get_cod() - Returns the type of message that the object
//contains
///////////////////////////////////////////////////////////////////////////////
int Trans_data::get_cod()
{
  return cod;
}


///////////////////////////////////////////////////////////////////////////////
//int Trans_data::get_cod() -  Returns the information saved in one of the
//first two positions of the message
///////////////////////////////////////////////////////////////////////////////
int Trans_data::get_info(int position)
{
  return info[position];
}

///////////////////////////////////////////////////////////////////////////////
//void Consensus::reset_d_dav_y() - sets values of d, d_av and y to 0
//useful in case of instability
///////////////////////////////////////////////////////////////////////////////
void Consensus::reset_d_dav_y()
{
  d[0] = 0;
  d[1] = 0;
  d_av[0] = 0;
  d_av[1] = 0;
  y[0] = 0;
  y[1] = 0;
}

///////////////////////////////////////////////////////////////////////////////
//Consensus::Consensus(float L_, float o_, float k_[]) - constructor
//sets the values of L, o and k when an object of this class is declared
///////////////////////////////////////////////////////////////////////////////
Consensus::Consensus(float L_, float o_, float k_[]) {
  L = L_;
  o = o_;
  k[0] = k_[0];
  k[1] = k_[1];
}

///////////////////////////////////////////////////////////////////////////////
//void Consensus::get_average(float av[]) - outputs the value of average
//of d = d[0] + d[1] the value is sent by reference
///////////////////////////////////////////////////////////////////////////////
void Consensus::get_average(float av[]) {
  av[0] = d_av[0];
  av[1] = d_av[1];
}

///////////////////////////////////////////////////////////////////////////////
//void Consensus::get_d(float di[]) - outputs the value of d[0] and d[1]
//values are sent by reference
///////////////////////////////////////////////////////////////////////////////
void Consensus::get_d(float di[]) {
  di[0] = d[0];
  di[1] = d[1];
}

///////////////////////////////////////////////////////////////////////////////
//float Consensus::get_o() - returns the value of o
///////////////////////////////////////////////////////////////////////////////
float Consensus::get_o() {
  return o;
}


///////////////////////////////////////////////////////////////////////////////
//void Consensus::update_k(float k_new[]) - updates the values of k
//from calibration
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_k(float k_new[]) {
  k[0] = k_new[0];
  k[1] = k_new[1];
}

///////////////////////////////////////////////////////////////////////////////
//void Consensus::update_L(float L_new) - updates the values of L, used 
//every time there is a change in the occupancy state
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_L(float L_new) {
  L = L_new;
}

///////////////////////////////////////////////////////////////////////////////
//void Consensus::update_o(float o_new) - updates the values of o, from 
//calibration
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_o(float o_new) {
  o = o_new;
}


///////////////////////////////////////////////////////////////////////////////
//void Consensus::update_average(float dj[]) - computes the average of d using
//the value d from the object and dj from the other luminaire state and updates
//the variable d_av[]. it also computes y[] - Lagrange  multiplier
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_average(float dj[]) {
  d_av[0] = (d[0] + dj[1]) / 2;
  d_av[1] = (d[1] + dj[0]) / 2;
  y[0] += RHO * (d[0] - d_av[0]);
  y[1] += RHO * (d[1] - d_av[1]);
}

///////////////////////////////////////////////////////////////////////////////
//bool Consensus::is_feasible(float di[]) - computes the feasibility check
//for each node. The feasibility check corresponds to verify if the possible 
//solution di verifies the inequality:di>0;di<MAX_D;diki+djkj<L-o 
//here we also include a TOLERANCE
///////////////////////////////////////////////////////////////////////////////
bool Consensus::is_feasible(float di[]) {
  if (di[0] < -TOLERANCE)
    return false;
  if (di[0] > MAX_D + TOLERANCE)
    return false;
  if (di[0]*k[0] + di[1]*k[1] < L - o - TOLERANCE)
    return false;

  return true;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::compute_cost(float di[]) - computes the cost from the cost 
//function defined to obtain the primal iteration. this function comes from
//the global optimization problem and then decomposed. in this case, we used 
//the global cost function f(d)=cTd+dT.Q.d
///////////////////////////////////////////////////////////////////////////////
float Consensus::compute_cost(float di[]) {
  float error[] = {di[0] - d_av[0], di[1] - d_av[1]};
  float cost = C_O * di[0] + y[0] * error[0] + y[1] * error[1];
  cost += RHO / 2 * (error[0] * error[0] + error[1] * error[1]) + di[0] * di[0] * Q;
  return cost;
}


///////////////////////////////////////////////////////////////////////////////
//void Consensus::primal_solver() - computes the solution for the optimization
//problem regarding the value of d[0]. first checks if the solution is inside 
//the domain and performs the feasibility check. then checks the 
//linear_boundary ILB, upper_boundary DUB, lower_boundary DLB and the 
//intersections ILB with DUB and ILB with DLB
///////////////////////////////////////////////////////////////////////////////
void Consensus::primal_solver() {
  float cost_best = 10000000;

  //              Unconstrained minimum               //
  d[0] = -(C_O + y[0] - d_av[0] * RHO) / (RHO + Q);
  d[1] = -(y[1] - d_av[1] * RHO) / RHO;
  if (Consensus::is_feasible(d))
    return;

  //          Minimum constrained to boundaries      //
  cost_best = Consensus::linear_boundary(d, cost_best);
  cost_best = Consensus::upper_boundary(d, cost_best);
  cost_best = Consensus::lower_boundary(d, cost_best);
  cost_best = Consensus::linear_and_upper_boundary(d, cost_best);
  cost_best = Consensus::linear_and_lower_boundary(d, cost_best);

  return;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::linear_boundary(float di[], float cost_best) - computes the
//solution at the illuminance boundary ILB, checks if it is feasible and sees
//if it is a better solution than the previous one (compares the costs and
//chooses the lowest)
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_boundary(float di[], float cost_best) {
  float d_aux[2];

  d_aux[0] = k[1] * k[1] * (C_O + y[0] - RHO * d_av[0]) + k[0] * RHO * (o - L) + k[0] * k[1] * (d_av[1] * RHO - y[1]);
  d_aux[0] = -d_aux[0] / (k[1] * k[1] * (Q + RHO) + k[0] * k[0] * RHO);

  d_aux[1] = k[1] * (L - o + (k[1] * (y[1] - d_av[1] * RHO)) / RHO + (k[1] * (C_O + y[0] - d_av[0] * RHO)) / (Q + RHO));
  d_aux[1] = d_aux[1] / (RHO * (k[0] * k[0] / (Q + RHO) + k[1] * k[1] / RHO)) - (y[1] - d_av[1] * RHO) / RHO;

  if (Consensus::is_feasible(d_aux)) {
    float cost = Consensus::compute_cost(d_aux);
    if (cost < cost_best) {
      di[0] = d_aux[0];
      di[1] = d_aux[1];
      return cost;
    }
  }
  return cost_best;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::lower_boundary(float di[], float cost_best) - computes the
//solution at the lower diming boundary. checks if it is feasible and sees if
//it is a better solution than the previous one (compares the costs and
//chooses the lowest)
///////////////////////////////////////////////////////////////////////////////
float Consensus::lower_boundary(float di[], float cost_best) {
  float d_aux[2];

  d_aux[0] = 0;
  d_aux[1] = -(y[1] - d_av[1] * RHO) / RHO;

  if (Consensus::is_feasible(d_aux)) {
    float cost = Consensus::compute_cost(d_aux);
    if (cost < cost_best) {
      di[0] = d_aux[0];
      di[1] = d_aux[1];
      return cost;
    }
  }
  return cost_best;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::upper_boundary(float di[], float cost_best) - computes the
//solution at the upper diming boundary. checks if it is feasible and sees if
//it is a better solution than the previous one (compares the costs and
//chooses the lowest)
///////////////////////////////////////////////////////////////////////////////
float Consensus::upper_boundary(float di[], float cost_best) {
  float d_aux[2];

  d_aux[0] = MAX_D;
  d_aux[1] = -(y[1] - d_av[1] * RHO) / RHO;

  if (Consensus::is_feasible(d_aux)) {
    float cost = Consensus::compute_cost(d_aux);
    if (cost < cost_best) {
      di[0] = d_aux[0];
      di[1] = d_aux[1];
      return cost;
    }
  }
  return cost_best;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::linear_and_lower_boundary(float di[], float cost_best) -
//computes the solution at the intersection of ILB with lower diming boundary 
//DLB. checks if it is feasible and sees if it is a better solution than 
//the previous one (compares the costs and chooses the lowest)
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_and_lower_boundary(float di[], float cost_best) {
  float d_aux[2];

  d_aux[0] = 0;
  d_aux[1] = (L - o) / k[1];

  if (Consensus::is_feasible(d_aux)) {
    float cost = Consensus::compute_cost(d_aux);
    if (cost < cost_best) {
      di[0] = d_aux[0];
      di[1] = d_aux[1];
      return cost;
    }
  }
  return cost_best;
}


///////////////////////////////////////////////////////////////////////////////
//float Consensus::linear_and_upper_boundary(float di[], float cost_best) -
//computes the solution at the intersection of ILB with upper diming boundary
//DLB. checks if it is feasible and sees if it is a better solution than
//the previous one (compares the costs and chooses the lowest)
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_and_upper_boundary(float di[], float cost_best) {
  float d_aux[2];

  d_aux[0] = MAX_D;
  d_aux[1] = (L - o - MAX_D * k[0]) / k[1];

  if (Consensus::is_feasible(d_aux)) {
    float cost = Consensus::compute_cost(d_aux);
    if (cost < cost_best) {
      di[0] = d_aux[0];
      di[1] = d_aux[1];
      return cost;
    }
  }
  return cost_best;
}


long unsigned int Time_PID = 0;
Trans_data sam, sam_2, sam_3;
boolean calibrado = 0;
boolean converg = 0;
float k[2], o;
float med[3];
Consensus Consens(0, 0, 0);
float L_new;
float d[2];
float d_ant[2];
float d_outro[2], d_outro_ant[2];
float d_av[2];
int interrupt_value;
byte erro_i2c = 0;
boolean resend = 0;
boolean sample_update = 0;
byte progress = 0;
boolean update_cons = 0;
boolean send_cons = 0;
boolean receiv_cons = 0;
//-------------------------------------------
// Controller variables
float Vout = 0;
float K1 = Kp * Ki * H / 2.0;
float Ka = 1;

float err, i, p;
float u = 0;
float u_sat = 0;
float i_ant = 0;
float err_ant = 0;
float aux;

float ff_in = 0;

// Simulator variables
float Vi = 0;
float Vf = 0;
float Vsim = 0;
float tau = B_TAU;


float R2;
float L;
float xd = 0;
//-------------------------------------------


///////////////////////////////////////////////////////////////////////////////
//ISR(TIMER2_COMPA_vect) - Timer2 interrupt handler - Activates the flags for
//updating the exit and error detection.
//This interrupt is called every 1ms (100Hz)
///////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect) {
  if (calibrado == 1)
  {
    sample_update = 1;
    if (progress == 0)
    {
      send_cons = 1;
      update_cons = 1;
    }
    else
    {
      progress = 0;
    }
  }
  if (erro_i2c != 0)
    resend = 1;
}


///////////////////////////////////////////////////////////////////////////////
//float convert_to_lux(float tension) - Converts the received tension read from
//the sensor into Lux.
///////////////////////////////////////////////////////////////////////////////
float convert_to_lux(float tension) {
  float alpha, beta;
  alpha = R1 * VCC / tension - R1;
  beta = log10(alpha) - B;
  alpha = pow(10, beta / M);
  return alpha;
}

///////////////////////////////////////////////////////////////////////////////
//void identification() - computes gains and disturbances. uses the values
//measured during calibration to compute k and o
///////////////////////////////////////////////////////////////////////////////
void identification() {
  float out;
  float I_i, I_f;

  out = med[0] * VCC / (MAX_READ + 1);
  o = convert_to_lux(out);

  out = med[1] * VCC / (MAX_READ + 1);
  I_f = convert_to_lux(out);
  k[0] = (I_f - o) / (MAX_WRITE - IN_I);
  if (k[0] < 0)
    k[0] = 0;

  out = med[2] * VCC / (MAX_READ + 1);
  I_f = convert_to_lux(out);
  k[1] = (I_f - o) / (MAX_WRITE - IN_I);
  if (k[1] < 0)
    k[1] = 0;
}


///////////////////////////////////////////////////////////////////////////////
//void calibration_procedure() - processes calibration. decodes messages received
//related to calibration, turning ON or OFF the LED and measuring the SENSOR.
//sends responses to continue the procedure until the gains and disturbances
//are calculated
///////////////////////////////////////////////////////////////////////////////
void calibration_procedure() {
  //Arduino 1 turns LED OFF
  if (sam_2.get_info(0) == 0 && sam.get_info(0) != 1)
  {
    analogWrite(LED, IN_I);
    sam.put_cod(I2C_CAL);
    sam.int_in_pos(0, 1);
    erro_i2c = sam.envia();
  }

  //Arduino 2 turns LED ON and reads own influence
  else if (sam_2.get_info(0) == 1 && sam.get_info(0) == 0)
  {
    analogWrite(LED, MAX_WRITE);
    delay(1000);
    med[1] = analogRead(SENSOR);
    sam.put_cod(I2C_CAL);
    sam.int_in_pos(0, 2);
    erro_i2c = sam.envia();
  }

  //Arduino 1 reads other influence and turns LED ON
  else if (sam_2.get_info(0) == 2 && sam.get_info(0) == 1)
  {
    med[2] = analogRead(SENSOR);
    analogWrite(LED, MAX_WRITE);
    sam.put_cod(I2C_CAL);
    sam.int_in_pos(0, 3);
    erro_i2c = sam.envia();
  }

  //Arduino 2 turns LED OFF and reads other influence
  else if (sam_2.get_info(0) == 3 && sam.get_info(0) == 2)
  {
    analogWrite(LED, IN_I);
    delay(1000);
    med[2] = analogRead(SENSOR);
    sam.put_cod(I2C_CAL);
    sam.int_in_pos(0, 4);
    erro_i2c = sam.envia();
  }

  //Arduino 1 reads own influence, turns LED OFF, reads disturbances and computes K and O
  else if (sam_2.get_info(0) == 4 && sam.get_info(0) == 3)
  {
    med[1] = analogRead(SENSOR);
    analogWrite(LED, IN_I);
    delay(1000);
    sam.put_cod(I2C_CAL);
    sam.int_in_pos(0, 5);
    erro_i2c = sam.envia();
    med[0] = analogRead(SENSOR);
    identification();
    sam.put_cod(31);
    sam.int_in_pos(1, 1023);
    Consens.update_k(k);
    Consens.update_o(o);

    sam_3.put_cod(I2C_RASP);
    sam_3.int_in_pos(0, RASP_CONST);
    sam_3.int_in_pos(1, L_Bound_H);
    sam_3.int_in_pos(2, L_Bound_L);
    sam_3.int_in_pos(3, (int) (Consens.get_o()));
    sam_3.envia();


    delay(1000);
    calibrado = 1;
  }

  //Arduino 2 reads disturbances and computes K and O
  else if (sam_2.get_info(0) == 5 && sam.get_info(0) == 4)
  {
    med[0] = analogRead(SENSOR);
    identification();
    sam.put_cod(31);
    sam.int_in_pos(1, 1023);
    Consens.update_k(k);
    Consens.update_o(o);

    sam_3.put_cod(I2C_RASP);
    sam_3.int_in_pos(0, RASP_CONST);
    sam_3.int_in_pos(1, L_Bound_H);
    sam_3.int_in_pos(2, L_Bound_L);
    sam_3.int_in_pos(3, (int) (Consens.get_o()));
    sam_3.envia();

    delay(1000);
    calibrado = 1;
  }

  //Waits 50 milliseconds for next cicle
  else
  {
    delay(50);
  }

}


///////////////////////////////////////////////////////////////////////////////
// int deadzone(float x,int xmin,int xmax)
// Creates the effect of a deadzone for the actuator
// It avoids actions on the system when the actuator input is negible
///////////////////////////////////////////////////////////////////////////////
float deadzone(float x, float xmin, float xmax)
{ float y;
  if ( x >= xmax )
    y = x - xmax;
  else if (x <= xmin)
    y = x + xmin;
  else
    y = 0;
  return y;
}


///////////////////////////////////////////////////////////////////////////////
//void update_out() - update of the LED. performs the simulator and PID controller
//and changes the value of the LED. sends data to be read by the server
///////////////////////////////////////////////////////////////////////////////
void update_out()
{
  // simulator
  aux = (millis() - Time_PID) * 0.001;
  Vsim = Vf - (Vf - Vi) * exp(-aux / tau);
  Vout = analogRead(SENSOR) * VCC / MAX_READ;  // Read value

  // compute the fb_in
  err = convert_to_lux(Vsim) - convert_to_lux(Vout);
  err = deadzone(err, -2.00, 0);
  p = Kp * err;
  i = i_ant + K1 * (err + err_ant) + Ka * (u_sat - u);

  // compute input
  u = ff_in + i;
  u_sat = constrain(u, 0, MAX_WRITE);
  analogWrite(LED, u_sat);            // Write value

  i_ant = i;
  err_ant = err;

  R2 = R1 * VCC / Vout - R1;
  L = pow(10, (log10(R2) - B) / M);
  if (Vout == 0)
    L = 0;


  // Messages to RaspberryPi
  int occ = 0;
  if (L_new == L_Bound_H) {
    occ = 1;
  }
  else {
    occ = 0;
  }

  sam_3.put_cod(I2C_RASP);
  sam_3.int_in_pos(0, occ);
  sam_3.int_in_pos(1, (int) (u_sat));
  sam_3.int_in_pos(2, (int) (L + 0.5));
  sam_3.int_in_pos(3, (int) (xd + 0.5));
  sam_3.envia();

}




float convergence(float d, float d_ant) {
  float result_converg;
  result_converg = d - d_ant;
  if (result_converg < 0) {
    return -result_converg;
  }
  else {
    return result_converg;
  }
}



void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  pinMode(IN_TERRUPT, INPUT);
  pinMode(LED, OUTPUT);
  Wire.begin(OWN_ADD);
  Wire.onReceive(receiveEvent); //event handler
  TWAR = (OWN_ADD << 1) | 1; // enable broadcasts to be received


  cli(); //stop interrupts
  TCCR1A = 0;// clear register
  TCCR1B = 0;// clear register
  TCNT1 = 0;//reset counter
  OCR1A = 624; //must be <65536
  // = (16*10^6) / (1*1024) – 1
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 1024
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //allow interrupts


  //Starts as Arduino 2
  sam.put_cod(I2C_CAL);
  sam.int_in_pos(0, 0);
  sam.envia();
}



void loop() {
  interrupt_value = digitalRead(IN_TERRUPT);
  if (interrupt_value == HIGH) {
    L_new = L_Bound_L;
    converg = 0;
  }
  if (interrupt_value == LOW) {
    L_new = L_Bound_H;
    converg = 0;
  }



  //Verifies if the other arduino restarted. If other has restarted, this becomes Arduino 1
  if (sam_2.get_cod() == I2C_CAL && sam_2.get_info(0) == 0)
    calibrado = 0;

  if (sam_2.get_cod() == I2C_CAL && calibrado == 0)
    calibration_procedure();

  if (sample_update == 1)
  {
    update_out();
    sample_update = 0;
  }



  //Runs the distributed control process if calibrated
  if (calibrado == 1) {
    if (update_cons == 0 && send_cons == 0)
    {
      Consens.update_L(L_new);
      Consens.primal_solver();
      Consens.get_d(d);
      update_cons = 1;
      send_cons = 1;
    }

    if (send_cons == 1 && update_cons == 1)
    {
      delay(1);
      sam.put_cod(I2C_CON);
      sam.int_in_pos(0, (int) (d[0] + 0.5));
      sam.int_in_pos(1, (int) (d[1] + 0.5));
      erro_i2c = sam.envia();
      progress += 1;
      send_cons = 0;
    }

    if (receiv_cons == 1 && send_cons == 0 && update_cons == 1)
    {
      receiv_cons = 0;
      d_outro[0] = sam_2.get_info(0);
      d_outro[1] = sam_2.get_info(1);
      Consens.update_average(d_outro);
      if (convergence(d[0], d_ant[0]) < 2.0) {
        converg = 1;
      }
      if (converg == 0) {
        Time_PID = millis();
      }
      if ((d[0] == 255 && d_ant[0] == 255) || (d_outro[0] == 255 && d_outro_ant[0] == 255)) {
        Consens.reset_d_dav_y();
      }
      ff_in = d[0];
      xd = k[0] * d[0] + k[1] * d[1] + o;
      R2 = pow(xd, M) * pow(10, B); // Compute new LDR resistance
      Vf = VCC * R1 / (R1 + R2);    // Compute the desired final voltage
      tau = M_TAU * Vf + B_TAU;     // Compute the new tau value
      Vi = Vsim;          // Update initial voltage

      d_ant[0] = d[0];
      d_ant[1] = d[1];
      d_outro_ant[0] = d_outro[0];
      d_outro_ant[1] = d_outro[1];
      update_cons = 0;
    }

  }


  if (resend == 1)
  {
    erro_i2c = sam.envia();
    resend = 0;
  }
}



///////////////////////////////////////////////////////////////////////////////
//void receiveEvent(int howMany) - I2C interrupt handler. Reads the messages
//received and saves the ones related to calibration and consensus.
///////////////////////////////////////////////////////////////////////////////
void receiveEvent(int howMany) {
  byte c[4];
  int i = 0;
  while (Wire.available() > 0)
  {
    c[i] = Wire.read();
    i++;
  }
  if ( i < I2C_RASP)
  {
    sam_2.recebe(c, i);
    if (i == I2C_CON)
    {
      receiv_cons = 1;
    }
  }
}
