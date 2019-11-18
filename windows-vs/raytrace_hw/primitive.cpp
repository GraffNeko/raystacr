#include"primitive.h"
#include<sstream>
#include<cstdio>
#include<string>
#include<cmath>
#include<iostream>
#include<cstdlib>
#include <algorithm>
#define ran() ( double( rand() % 32768 ) / 32768 )

const int BEZIER_MAX_DEGREE = 5;
const int Combination[BEZIER_MAX_DEGREE + 1][BEZIER_MAX_DEGREE + 1] =
{	0, 0, 0, 0, 0, 0,
	1, 1, 0, 0, 0, 0,
	1, 2, 1, 0, 0, 0,
	1, 3, 3, 1, 0, 0,
	1, 4, 6, 4, 1, 0,
	1, 5, 10,10,5, 1
};

const int MAX_COLLIDE_TIMES = 10;
const int MAX_COLLIDE_RANDS = 10;


std::pair<double, double> ExpBlur::GetXY()
{
	double x,y;
	x = ran();
	x = pow(2, x)-1;
	y = rand();
	return std::pair<double, double>(x*cos(y),x*sin(y));
}

Material::Material() {
	color = absor = Color();
	refl = refr = 0;
	diff = spec = 0;
	rindex = 0;
	drefl = 0;
	texture = NULL;
	blur = new ExpBlur();
}

void Material::Input( std::string var , std::stringstream& fin ) {
	if ( var == "color=" ) color.Input( fin );
	if ( var == "absor=" ) absor.Input( fin );
	if ( var == "refl=" ) fin >> refl;
	if ( var == "refr=" ) fin >> refr;
	if ( var == "diff=" ) fin >> diff;
	if ( var == "spec=" ) fin >> spec;
	if ( var == "drefl=" ) fin >> drefl;
	if ( var == "rindex=" ) fin >> rindex;
	if ( var == "texture=" ) {
		std::string file; fin >> file;
		texture = new Bmp;
		texture->Input( file );
	}
	if ( var == "blur=" ) {
		std::string blurname; fin >> blurname;
		if(blurname == "exp")
			blur = new ExpBlur();
	}
}

Primitive::Primitive() {
	sample = rand();
	material = new Material;
	next = NULL;
}

Primitive::Primitive( const Primitive& primitive ) {
	*this = primitive;
	material = new Material;
	*material = *primitive.material;
}

Primitive::~Primitive() {
	delete material;
}

void Primitive::Input( std::string var , std::stringstream& fin ) {
	material->Input( var , fin );
}

Sphere::Sphere() : Primitive() {
	De = Vector3( 0 , 0 , 1 );
	Dc = Vector3( 0 , 1 , 0 );
}

void Sphere::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "R=" ) fin >> R;
	if ( var == "De=" ) De.Input( fin );
	if ( var == "Dc=" ) Dc.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Sphere::Collide( Vector3 ray_O , Vector3 ray_V ) {
	ray_V = ray_V.GetUnitVector();
	Vector3 P = ray_O - O;
	double b = -P.Dot( ray_V );
	double det = b * b - P.Module2() + R * R;
	CollidePrimitive ret;

	if ( det > EPS ) {
		det = sqrt( det );
		double x1 = b - det  , x2 = b + det;

		if ( x2 < EPS ) return ret;
		if ( x1 > EPS ) {
			ret.dist = x1;
			ret.front = true;
		} else {
			ret.dist = x2;
			ret.front = false;
		} 
	} else 
		return ret;

	ret.C = ray_O + ray_V * ret.dist;
	ret.N = ( ret.C - O ).GetUnitVector();
	if ( ret.front == false ) ret.N = -ret.N;
	ret.isCollide = true;
	ret.collide_primitive = this;
	return ret;
}

Color Sphere::GetTexture(Vector3 crash_C) {
	Vector3 I = ( crash_C - O ).GetUnitVector();
	double a = acos( -I.Dot( De ) );
	double b = acos( std::min( std::max( I.Dot( Dc ) / sin( a ) , -1.0 ) , 1.0 ) );
	double u = a / PI , v = b / 2 / PI;
	if ( I.Dot( Dc * De ) < 0 ) v = 1 - v;
	return material->texture->GetSmoothColor( u , v );
}


void Plane::Input( std::string var , std::stringstream& fin ) {
	if ( var == "N=" ) N.Input( fin );
	if ( var == "R=" ) fin >> R;
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Plane::Collide( Vector3 ray_O , Vector3 ray_V ) {
	ray_V = ray_V.GetUnitVector();
	N = N.GetUnitVector();
	double d = N.Dot( ray_V );
	CollidePrimitive ret;
	if ( fabs( d ) < EPS ) return ret;
	double l = ( N * R - ray_O ).Dot( N ) / d;
	if ( l < EPS ) return ret;

	ret.dist = l;
	ret.front = ( d < 0 );
	ret.C = ray_O + ray_V * ret.dist;
	ret.N = ( ret.front ) ? N : -N;
	ret.isCollide = true;
	ret.collide_primitive = this;
	return ret;
}

Color Plane::GetTexture(Vector3 crash_C) {
	double u = crash_C.Dot( Dx ) / Dx.Module2();
	double v = crash_C.Dot( Dy ) / Dy.Module2();
	return material->texture->GetSmoothColor( u , v );
}

void Square::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Primitive::Input( var , fin );
}

CollidePrimitive Square::Collide( Vector3 ray_O , Vector3 ray_V ) {//����ε���ײ���
	CollidePrimitive ret;
	//NEED TO IMPLEMENT
	//�Ƿ����ı��η�������ײ
	ray_V = ray_V.GetUnitVector();
	Vector3 N = (Dx * Dy).GetUnitVector();
	double d = N.Dot(ray_V);
	if (fabs(d) < EPS) {//ƽ��û�н���
		return ret;
	}
	double theT = (O - ray_O).Dot(N) / d;//�õ�t��ֵ
	//printf("%f \n", theT);
	if (theT < EPS) {  //���߷��򣬲�����ײ
		return ret;
	}

	Vector3 thePoint = ray_O + ray_V * theT;//�����ƽ��Ľ���
	Vector3 theVec = thePoint - O;

	if (fabs( theVec.Dot(Dx) / Dx.Module2() ) -0.5 < EPS && fabs( theVec.Dot(Dy) / Dy.Module2()) -0.5 < EPS) //�����ƽ��Ľ����ھ����ڲ�
	{
		//printf("%f %f\n", judgeX, judgeY);
		ret.dist = theT;
		ret.front = (d < 0);
		ret.C = thePoint;
		ret.N = (ret.front) ? N : -N;
		ret.isCollide = true;
		ret.collide_primitive = this;
		return ret;
	}
	return ret;
}

Color Square::GetTexture(Vector3 crash_C) {
	double u = (crash_C - O).Dot( Dx ) / Dx.Module2()/2  + 0.5;
	double v = (crash_C - O).Dot( Dy ) / Dy.Module2()/2 + 0.5;
	return material->texture->GetSmoothColor( u , v );
}

void Cylinder::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O1=" ) O1.Input( fin );
	if ( var == "O2=" ) O2.Input( fin );
	if ( var == "R=" ) fin>>R; 
	Primitive::Input( var , fin );
}

CollidePrimitive Cylinder::Collide( Vector3 ray_O , Vector3 ray_V ) {
	CollidePrimitive ret;
	//NEED TO IMPLEMENT
	//����ֱ��ֱ�ߵľ��루���߷�������ߵľ��룩
	ray_V = ray_V.GetUnitVector();
	Vector3 centerAxis = O2 - O1;
	centerAxis = centerAxis.GetUnitVector();

	Vector3 o1ToO = ray_O - O1;
	Vector3 A = ray_V * centerAxis;   
	Vector3 B = o1ToO * centerAxis;

	double a = A.Dot(A); 
	double b = 2 * A.Dot(B); 
	double c = B.Dot(B) - R * R;
	double det = b * b - 4 * a * c;//����ķ�������

	if (det > EPS) {//������������
		det = sqrt(det);
		double x1 = (-b - det) / (2 * a), x2 = (-b + det) / (2 * a); //����ҵ�����
		//x1��������ӵ���ĵ� , x2��������ӵ�Զ�ĵ�
		if (x2 < EPS) {//���߷����෴�ˣ�û�н���
			return ret;
		}
		//�жϽ����λ��  ����1 ����2
		Vector3 collide1 = ray_O + x1 * ray_V; 
		Vector3 collide2 = ray_O + x2 * ray_V;
		//O1 O2����ײ��1�������Լ������ߵ�ͶӰ
		Vector3 o1ToCollide1 = collide1 - O1;
		Vector3 o2ToCollide1 = collide1 - O2;
		double o1ToCollide1InCenterAxis = o1ToCollide1.Dot(centerAxis);
		double o2ToCollide1InCenterAxis = o2ToCollide1.Dot(centerAxis);
		//O1 O2����ײ��2�������Լ������ߵ�ͶӰ
		Vector3 o1ToCollide2 = collide2 - O1;
		Vector3 o2ToCollide2 = collide2 - O2;
		double o1ToCollide2InCenterAxis = o1ToCollide2.Dot(centerAxis);
		double o2ToCollide2InCenterAxis = o2ToCollide2.Dot(centerAxis);
		//�ж���ײ��1 2 �ľ���λ��
		if (o1ToCollide1InCenterAxis < 0 && o2ToCollide1InCenterAxis < 0) {        //��ײ��1��O1֮��
			if (o1ToCollide2InCenterAxis < 0 && o2ToCollide2InCenterAxis < 0) 
				return ret; //��ײ��2��O1֮��   ˵����Բ���޽���
			
			//��һ����������Բ����Բ�潻��   ֻ�������һ�����㼴��
			Vector3 N = - centerAxis;
			double d = ray_V.Dot(N);
			double theT = (O1 - ray_O).Dot(N) / d;  //t��ֵ
			if (theT < EPS) {
				return ret;
			}
			ret.dist = theT;
			ret.front = true;
			ret.C = ray_O + ray_V * ret.dist;
			ret.N = N;
			ret.isCollide = true;
			ret.collide_primitive = this;
		}
		else if (o1ToCollide1InCenterAxis >= 0 && o2ToCollide1InCenterAxis <= 0) { //��ײ��1��O1  O2֮�� ���������Ҫ�ҵĵ�
			ret.dist = x1;
			ret.front = true;
			ret.C = ray_O + ray_V * ret.dist;
			ret.N = o1ToCollide2.Ortho(centerAxis);
			ret.isCollide = true;
			ret.collide_primitive = this;
		}
		else {                              //��ײ��1��O2֮��
			if (o1ToCollide2InCenterAxis > 0 && o2ToCollide2InCenterAxis > 0) 
				return ret; //��ײ��2��O2֮��
			//��һ������������Բ�潻��   ֻ�������һ�����㼴��
			Vector3 N = centerAxis;
			double d = ray_V.Dot(N);
			double theT = (O2 - ray_O).Dot(N) / d;//t��ֵ
			if (theT < EPS) {
				return ret;
			}
			ret.dist = theT;
			ret.front = true;
			ret.C = ray_O + ray_V * ret.dist;
			ret.N = N;
			ret.isCollide = true;
			ret.collide_primitive = this;
		}
	}

	return ret;
}

Color Cylinder::GetTexture(Vector3 crash_C) {

	Vector3 o2o1 = O1 - O2;
	Vector3 o2c = crash_C - O2;

	double u = o2c.Dot(o2o1) / o2o1.Module2(); //y/h;

	Vector3 r = o2o1.GetAnVerticalVector();
	o2o1 = o2o1.GetUnitVector();
	Vector3 r1 = o2c.Ortho(o2o1);
	r1 = r1.GetUnitVector();
	double radian = acos(r.Dot(r1));

	double judge = o2o1.Dot(r * r1);
	if (judge < 0) radian = 2 * PI - radian;  //�ж��Ƿ����180��
	double v = radian / (2 * PI);             //radian/(2*PI)

	return material->texture->GetSmoothColor(u, v);
}

void Bezier::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O1=" ) O1.Input( fin );
	if ( var == "O2=" ) O2.Input( fin );
	if ( var == "P=" ) {
		degree++;
		double newR, newZ;
		fin>>newZ>>newR;
		R.push_back(newR);
		Z.push_back(newZ);
	}
	if ( var == "Cylinder" ) {
		double maxR = 0;
		for(int i=0;i<R.size();i++)
			if(R[i] > maxR)
				maxR = R[i];
		boundingCylinder = new Cylinder(O1, O2, maxR);
		N = (O1 - O2).GetUnitVector();
		Nx = N.GetAnVerticalVector();
		Ny = N * Nx;
	}
	Primitive::Input( var , fin );
}

CollidePrimitive Bezier::Collide( Vector3 ray_O , Vector3 ray_V ) {
	CollidePrimitive ret;
	//NEED TO IMPLEMENT
	return ret;
}

Color Bezier::GetTexture(Vector3 crash_C) {
	double u = 0.5 ,v = 0.5;
	//NEED TO IMPLEMENT
	return material->texture->GetSmoothColor( u , v );
}

