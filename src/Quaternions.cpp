//============================================================================
// Name        : Quaternions.cpp
// Author      : Marc Casalprim
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <bitset>
#include <iostream>
#include <math.h>
using namespace std;

double PI=M_PI;

struct Attitude
{
	bool valid;
	double ra;
	double az;
	double dec;
	double el;
	double roll;
};


/* Quaternion class - MjR */

#ifndef _QUATERNIONS_H
#define _QUATERNIONS_H

template<class _Tp>
class Quaternion
{
public:
	Quaternion(void);
	~Quaternion();
	Quaternion(_Tp X, _Tp Y, _Tp Z, _Tp W);//: x(X),y(Y),z(Z),w(W) {}
	Quaternion(const Quaternion<_Tp>& p);
	/*void CreateFromAxisAngle(	const double &in_x,
						const double &in_y,
						const double &in_y,
						const double &in_radians);*/
	Quaternion<_Tp> operator*(const Quaternion<_Tp> & p);
	Quaternion<_Tp> operator=(const Quaternion<_Tp> & p);
	//std::ostream &operator <<(std:ostream &os, Quaternion const &q);
	Quaternion<_Tp> inv();
	Attitude toAtt();
	void print();
	_Tp x,y,z,w;
};

#endif




template<class _Tp>
Quaternion<_Tp>::Quaternion(void)
{
	x=0.0;y=0.0;z=0.0;w=0.0;
}

template<class _Tp>
Quaternion<_Tp>::~Quaternion()
{
}



template<class _Tp>
Quaternion<_Tp>::Quaternion(_Tp X, _Tp Y, _Tp Z, _Tp W)
{
	x=X;
	y=Y;
	z=Z;
	w=W;
}


template<class _Tp>
Quaternion<_Tp>::Quaternion(const Quaternion<_Tp>& p)
{
	x=p.x;
	y=p.y;
	z=p.z;
	w=p.w;
}

template<class _Tp>
Quaternion<_Tp> Quaternion<_Tp>::operator = (const Quaternion<_Tp>& p)
{
	w=p.w;
	x=p.x;
	y=p.y;
	z=p.z;
	return (*this);
}

// Note the particular order of multiplications and additions - see MjR's thesis for more details (appendices)
template<class _Tp>
Quaternion<_Tp> Quaternion<_Tp>::operator * (const Quaternion<_Tp>& p)
{
	_Tp q1 = x;
	_Tp q2 = y;
	_Tp q3 = z;
	_Tp q4 = w;
	_Tp p1 = p.x;
	_Tp p2 = p.y;
	_Tp p3 = p.z;
	_Tp p4 = p.w;
	return Quaternion(
			q4*p1+q3*p2-q2*p3+q1*p4,
			-q3*p1+q4*p2+q1*p3+q2*p4,
			q2*p1-q1*p2+q4*p3+q3*p4,
			-q1*p1-q2*p2-q3*p3+q4*p4);
}

template<class _Tp>
Quaternion<_Tp> Quaternion<_Tp>::inv()
{
	return Quaternion(-x,-y,-z,w);
}

template<class _Tp>
Attitude Quaternion<_Tp>::toAtt()
{
	_Tp qr=w;
	_Tp qi=x;
	_Tp qj=y;
	_Tp qk=z;
	_Tp zn = 1-2*(qk*qk+qj*qj);
	_Tp xa=1-2*(qi*qi+qj*qj);
	_Tp xb=2*(qr*qi+qj*qk); //2*()
	_Tp xn = -2*(qr*qj-qi*qk); //-2*(wy-zx)
	_Tp yn=2*(qr*qk+qj*qi); //2*(wz+xy)


	Attitude att;
	att.ra=atan2(yn,zn);
	att.roll=atan2(xb,xa);
	if (att.roll<0) att.roll+=2*PI;
	if (att.ra<0) att.ra+=2*PI;
	//cout << "xn = " << xn << "\n";
	att.dec=asin(xn);
	return att;
}

template<class _Tp>
void Quaternion<_Tp>::print()
{
cout << "\nQuaternion:\n";
cout << "qr = " << w << "\n";
cout << "qi = " << x << "\n";
cout << "qj = " << y << "\n";
cout << "qk = " << z << "\n";
cout << "RA = " << this->toAtt().ra*180./PI << " deg\n";
cout << "DEC = " << this->toAtt().dec*180./PI << " deg\n";
cout << "ROLL = " << this->toAtt().roll*180./PI << " deg\n";
}
/* End of quaternion class */



void test(){

	int yaw = 3 ;
	int pitch = -44;
	int roll = 0.1;

	int caso = 0;

	if(caso == 0){
		//Three rotations one after the other

		Quaternion<double> qYAW(0.0,0.0,sin(-yaw/2.0),cos(-yaw/2.0));
		Quaternion<double> qPITCH(0.0,sin(-pitch/2.0),0.0,cos(-pitch/2.0));
		Quaternion<double> qROLL(sin(-roll/2.0),0.0,0.0,cos(-roll/2.0));
		Quaternion<double> qConsecutiveRotations = qYAW*(qPITCH*qROLL);

		cout << "\nRotation Quaternion -roll * - pitch * - yaw\n";
		cout << "qr = " << qConsecutiveRotations.w << "\n";
		cout << "qi = " << qConsecutiveRotations.x << "\n";
		cout << "qj = " << qConsecutiveRotations.y << "\n";
		cout << "qk = " << qConsecutiveRotations.z << "\n";
		qConsecutiveRotations = qConsecutiveRotations.inv();
		cout << "Ra = " << qConsecutiveRotations.toAtt().ra*180./PI << " Dec = " << qConsecutiveRotations.toAtt().dec*180./PI << "\n";


		qYAW = Quaternion<double> (0.0,0.0,sin(-yaw/2.0),cos(-yaw/2.0));
		qPITCH = Quaternion<double> (0.0,sin(-pitch/2.0),0.0,cos(-pitch/2.0));
		qROLL = Quaternion<double> (0,0.0,0.0,1);
		 qConsecutiveRotations = (qYAW * qPITCH);

		qROLL = Quaternion<double> (sin(-roll/2.0),0.0,0.0,cos(-roll/2.0));
		qConsecutiveRotations = qROLL * qConsecutiveRotations;



		cout << "\nRotation Quaternion -pitch * -yaw and 0 roll. Then -roll\n";
		cout << "qr = " << qConsecutiveRotations.w << "\n";
		cout << "qi = " << qConsecutiveRotations.x << "\n";
		cout << "qj = " << qConsecutiveRotations.y << "\n";
		cout << "qk = " << qConsecutiveRotations.z << "\n";
		cout << "Ra = " << qConsecutiveRotations.toAtt().ra*180./PI << " Dec = " << qConsecutiveRotations.toAtt().dec*180./PI << "\n";


	}


}


Quaternion<double> DCM2Quat(double (&matrix)[3][3]){
	double trace = matrix [0][0] + matrix [1][1] +matrix [2][2];
	double sr;
	double sr2;
	double x,y,z,r;

	if(trace > 0){ // positive trace
		sr = sqrt(1 + trace);
		sr2 = 2*sr;
		x = (matrix[1][2] - matrix[2][1]) / sr2;
		y = (matrix[2][0] - matrix[0][2]) / sr2;
		z = (matrix[0][1] - matrix[1][0]) / sr2;
		r = 0.5* sr;
	}
	else{ //negative trace
		if((matrix[0][0] > matrix[1][1]) && (matrix[0][0] > matrix[2][2])){
			//Maximum Value at matrix[0][0]
		      sr  = sqrt( 1 + (matrix[0][0] - ( matrix[1][1] + matrix[2][2] )) );
		      sr2 = 2*sr;
		      x = 0.5 * sr;
		      y = ( matrix[1][0] + matrix[0][1] ) / sr2;
		      z = ( matrix[2][0] + matrix[0][2] ) / sr2;
		      r = ( matrix[1][2] - matrix[2][1] ) / sr2;
		}
		else if (matrix[1][1] > matrix[2][2]) {
		      // Maximum Value at matrix[1][1]
		      sr  = sqrt( 1 + (matrix[1][1] - ( matrix[2][2] + matrix[0][0] )) );
		      sr2 = 2*sr;
		      x = ( matrix[1][0] + matrix[0][1] ) / sr2;
		      y = 0.5 * sr;
		      z = ( matrix[1][2] + matrix[2][1] ) / sr2;
		      r = ( matrix[2][0] - matrix[0][2] ) / sr2;
		}
		else{
		      // Maximum Value at matrix[2][2]
		      sr  = sqrt( 1 + (matrix[2][2] - ( matrix[0][0] + matrix[1][1] )) );
		      sr2 = 2*sr;
		      x = ( matrix[2][0] + matrix[0][2] ) / sr2;
		      y = ( matrix[1][2] + matrix[2][1] ) / sr2;
		      z = 0.5 * sr;
		      r = ( matrix[0][1] - matrix[1][0] ) / sr2;
		}
	}
	return Quaternion<double> (x,y,z,r);

}

int calcQuaternions(Attitude equatorial,int cameraSerial,float YawLeft,float PitchLeft,float RollLeft,float YawRight,float PitchRight,float RollRight) {

	int ret = 0;

	// add quaternion support MjR 11/4/15
	// elementary quaternions used in determining the inertial attitude (see MjR's thesis)
	Quaternion<double> qRA(0.0,0.0,sin(equatorial.ra/2.0),cos(equatorial.ra/2.0));
	Quaternion<double> qDEC(0.0,sin(-equatorial.dec/2.0),0.0,cos(equatorial.dec/2.0));
	Quaternion<double> qROLL(sin(equatorial.roll/2.0),0.0,0.0,cos(equatorial.roll/2.0));



	// Elementary quaternions used in determining the quaternion from starcamera to gyro reference frame
	// the individual rotations must be provided by optical alignment of the gyros and star cameras
	// these angles depend on which camera is used.
	// angles in radians
	float dYaw = 0.0;
	float dPitch = 0.0;
	float dRoll = 0.0;
	cout << "Starcam alignments:\n";
	if (cameraSerial == 1){ // 1 is LEFT , 0 is RIGHT
		dYaw = YawLeft*PI/180.0;//-0.3605*PI/180.0;  // Arnab alignment numbers, see "Gyroscopes and Star Camera Measurements.xslx"
		dPitch = PitchLeft*PI/180.0;//46.6729*PI/180.0;
		dRoll = RollLeft*PI/180.0;//-2.1463*PI/180.0;
		cout << "YawLeft = " << YawLeft << " deg\n";
		cout << "PitchLeft = " << PitchLeft << " deg\n";
		cout << "RollLeft = " << RollLeft << " deg\n";
	} else {
		dYaw = YawRight*PI/180.0;//-0.0588*PI/180.0;
		dPitch = PitchRight*PI/180.0;//45.4472*PI/180.0;
		dRoll = RollRight*PI/180.0;//-0.2914*PI/180.0;
		cout << "YawRight = " << YawRight << "\n";
		cout << "PitchRight = " << PitchRight << "\n";
		cout << "RollRight = " << RollRight << "\n";
	}
	Quaternion<double> qdYaw(0.0,0.0,sin(-dYaw/2.0),cos(dYaw/2.0));
	Quaternion<double> qdPitch(0.0,sin(dPitch/2.0),0.0,cos(dPitch/2.0));
	Quaternion<double> qdRoll(sin(-dRoll/2.0),0.0,0.0,cos(dRoll/2.0));
	Quaternion<double> qI2Starcam = qROLL*(qDEC*qRA);
	Quaternion<double> qStarcam2Gyros = qdYaw*(qdPitch*qdRoll);
	Quaternion<double> qI2Gyros;
	qI2Gyros= qStarcam2Gyros * qI2Starcam;

	float tYaw = 3*PI/180.; //in radians
	float tPitch = -50*PI/180.;//PI/4.0;
	float tRoll = 2*PI/180.;

	Quaternion<double> qtYaw(0.0,0.0,sin(tYaw/2.0),cos(tYaw/2.0));
	Quaternion<double> qtPitch(0.0,sin(tPitch/2.0),0.0,cos(tPitch/2.0));
	Quaternion<double> qtRoll(sin(tRoll/2.0),0.0,0.0,cos(tRoll/2.0));

	Quaternion<double> qGyros2Telescope = qtRoll*(qtPitch*qtYaw);
	Quaternion<double> qI2Telescope = qGyros2Telescope*qI2Gyros;
	Attitude att = qI2Telescope.toAtt();
	/*cout << "qRA quaternion: " << qRAb << "\n";
	cout << "qDEC quaternion: " << qDECb << "\n";
	cout << "qROLL quaternion: " << qROLLb << "\n";
	cout << "qI2SC attitude quaternion: " << qI2SCb << "\n";
	cout << "qI2SC=(qr,qi,qj,qk) \n";
	cout << "qrb = " << qI2SCb.R_component_1() << "\n";
	cout << "qib = " << qI2SCb.R_component_2() << "\n";
	cout << "qjb = " << qI2SCb.R_component_3() << "\n";
	cout << "qkb = " << qI2SCb.R_component_4() << "\n";*/
	cout << "\nInertial attitude:\n";
	cout << "ROLL = " << equatorial.roll*180./PI << " deg\n";
	cout << "RA = " << equatorial.ra*180./PI << " deg\n";
	cout << "DEC = " << equatorial.dec*180./PI << " deg\n";
	cout << "\nInertial2Starcam quaternion:\n";
	cout << "qr = " << qI2Starcam.w << "\n";
	cout << "qi = " << qI2Starcam.x << "\n";
	cout << "qj = " << qI2Starcam.y << "\n";
	cout << "qk = " << qI2Starcam.z << "\n";
	//	cout << "\nInertial2Gyros quaternion:\n";
	//	cout << "qr = " << qI2Gyros.w << "\n";
	//	cout << "qi = " << qI2Gyros.x << "\n";
	//	cout << "qj = " << qI2Gyros.y << "\n";
	//	cout << "qk = " << qI2Gyros.z << "\n";
	cout << "\nInertial2Telescope quaternion:\n";
	cout << "qr = " << qI2Telescope.w << "\n";
	cout << "qi = " << qI2Telescope.x << "\n";
	cout << "qj = " << qI2Telescope.y << "\n";
	cout << "qk = " << qI2Telescope.z << "\n";
	cout << "\nTelescope attitude:\n";
	cout << "ROLL = " << att.roll*180./PI << " deg\n";
	cout << "RA = " << att.ra*180./PI << " deg\n";
	cout << "DEC = " << att.dec*180./PI << " deg\n";

	return ret;
}

int main() {

	Attitude equatorial;
	equatorial.roll=2*PI/180.;
	equatorial.ra=14.0*PI/180.; //in radians
	equatorial.dec=60.0*PI/180.;


	int cameraSerial=0; // 1 is LEFT , 0 is RIGHT

	float YawLeft=-0.3605;
	float PitchLeft=46.6729;
	float RollLeft=-2.1463;

	//	float YawRight=-0.0588;
	//	float PitchRight=45.4472;
	//	float RollRight=-0.2914;

	float YawRight=3;
	float PitchRight=50;
	float RollRight=2;

	//calcQuaternions(equatorial,cameraSerial,YawLeft,PitchLeft,RollLeft,YawRight,PitchRight,RollRight);
	//test();


	double eye[3][3] = {
	  { 1,0,0 },
	  { 0,1,0 },
	  { 0,0,1 }
	};

	double arnabMatrix [3][3] = {
		{0.686949,      0.00345102,   0.726697 },
		{-0.00237069,  0.999994,      -0.00250786 },
		{-0.726701,     0.0,               0.686953}

	};

	RollLeft=-2.1463*PI/180;
	RollLeft = -RollLeft;
	Quaternion<double> qR(sin(RollLeft/2.0),0.0,0.0,cos(RollLeft/2.0));

	Quaternion<double> toPrint = DCM2Quat(arnabMatrix);
	toPrint.print();
//	Quaternion<double> qToPrintRotated = (qR*toPrint);
//	qToPrintRotated.print();

	toPrint.inv().print();

	Quaternion<double> qStarcam2Gyros = (qR*toPrint.inv());
	qStarcam2Gyros.print();

	double dYaw = 359.802*PI/180;
	double dPitch = 46.6105*PI/180;
	double dRoll =  2.1463*PI/180;

	Quaternion<double> qdYawFabricatedQuaternion(0.0,0.0,sin(dYaw/2.0),cos(dYaw/2.0));
	Quaternion<double> qdPitchFabricatedQuaternion(0.0,sin(dPitch/2.0),0.0,cos(dPitch/2.0));
	Quaternion<double> qdRollFabricatedQuaternion(sin(dRoll/2.0),0.0,0.0,cos(dRoll/2.0));
	Quaternion<double> qFabricatedQuaternionStarcamToGyros = qdRollFabricatedQuaternion*qdPitchFabricatedQuaternion*qdYawFabricatedQuaternion;

	qFabricatedQuaternionStarcamToGyros.print();

	return 0;
}
