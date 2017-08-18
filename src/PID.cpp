#include "PID.h"
#include <iostream>
#include <limits>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error = i_error = d_error = 0;
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    dKp = Kp / 10;
    dKi = Ki / 10;
    dKd = Kd / 10;
    tuned = true;
    //tol = 0.008;
    current_round = 0;
    max_round = 250;
    best_err = std::numeric_limits<double>::max();
    sub_round = 0;
    acc_err = 0;
    acc_n = 50;
    state = -1;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    if (sub_round == 0)
        cout << "shit" << (int)tuned << " state:" << state << " "
            << current_round << " " << Kp << " " << Ki << " " << Kd
            << " " << dKp << " " << dKi << " " << dKd << " " << sub_round << endl;
    double ret = -Kp * p_error - Kd * d_error - Ki * i_error;
    return ret;
}

void PID::Twiddle(double err) {
    if (sub_round == 0)
        cout << "cte " << err << endl;
    if (tuned)
        return;
    if (current_round > max_round) {
        tuned = true;
        return;
    }
    acc_err += err * err;
    sub_round = (sub_round + 1) % acc_n;
    if (state != -1 && sub_round != 0) {
        return;
    }
    err = acc_err;
    acc_err = 0;

    double *param = NULL;
    double *dp = NULL;
    switch (cur_p_idx) {
        case (0):
            param = &Kp;
            dp = &dKp;
            break;
        case (1):
            param = &Ki;
            dp = &dKi;
            break;
        case (2):
            param = &Kd;
            dp = &dKd;
            break;
    }

    const double UP = 1.25;
    const double DOWN = 0.8;
    //state 0 +  1 check&- 2 check
    switch (state) {
        case (-1):
            *param += *dp;
            state = 0;
            sub_round = 0;
            break;
        case (0):
            state = 1;
            break;
        case (1):
            if (err < best_err) {
                *dp *= UP;
                best_err = err;
                cur_p_idx = (cur_p_idx + 1) % 3;
                d_error = i_error = 0;
                state = -1;
            } else {
                *param -= 2 * (*dp);
                state = 2;
            }
            break;
        case (2):
            if (err < best_err) {
                *dp *= UP;
                best_err = err;
            } else {
                *param += *dp;
                *dp *= DOWN;
            }
            cur_p_idx = (cur_p_idx + 1) % 3;
            d_error = i_error = 0;
            state = -1;
            break;
    }

    /*
    if (err < best_err) {
        best_err = err;
        *dp *= 1.1;
    } else {
        cout << "qqq " << (current_round / 2) % 3 << " " << (current_round / 3) % 2 << endl;
        if ((current_round / 3) % 2) {
            *param += (*dp);
            *dp *= 0.9;
        } else {
            *param -= 2 * (*dp);
        }
    }
    d_error = i_error = 0;
    cur_p_idx = (cur_p_idx + 1) % 3;
    */

    current_round++;
    //if (dKp + dKi + dKd < tol)
    //    tuned = true;
}

double PID::Speed(double sp) {
    if (!tuned)
        return sp / 10;
    return sp;
}
