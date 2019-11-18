#include"light.h"
#include<sstream>
#include<string>
#include<cmath>
#include<cstdlib>
#define ran() ( double( rand() % 32768 ) / 32768 )

Light::Light() {
	sample = rand();
	next = NULL;
	lightPrimitive = NULL;
}

void Light::Input( std::string var , std::stringstream& fin ) {
	if ( var == "color=" ) color.Input( fin );
}

void PointLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	Light::Input( var , fin );
}


double PointLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {  //点光源
	Vector3 V = O - C;
	double dist = V.Module();
	for ( Primitive* now = primitive_head ; now != NULL ; now = now->GetNext() )
	{
		CollidePrimitive tmp = now->Collide(C, V);

			Sphere * s = dynamic_cast<Sphere *>(now);  //球的soft shadow
			if (tmp.isCollide && s) {
				//实现软阴影
				Vector3 collisionV = tmp.C - C;  //视线方向
				collisionV = collisionV.GetUnitVector();
				double NprojectionInCollisionV = fabs(collisionV.Dot(tmp.N)); //投影
				return   1- NprojectionInCollisionV / tmp.N.Module();  //返回soft阴影的值
			}
			Square * square = dynamic_cast<Square *>(now);//正方形的soft shadow
			if (tmp.isCollide&&square)
			{
				double a = fabs( (tmp.C - square->O).Dot(square ->Dx)/ square->Dx.Module2() );
				double b = fabs((tmp.C - square->O).Dot(square->Dy) / square->Dx.Module2() );
				if (a <= 0.3 && b <= 0.3) {
					if (EPS < dist - tmp.dist) {
						return 0;
					}
					return 1;
				}
				else
				{
					if (EPS < dist - tmp.dist)
					{
						return ((a > b ? a : b) - 0.3) / 0.2;
					}
					return 1;
				}
			}
			if (EPS < dist - tmp.dist) {
				return 0;//没有软阴影
			}

		
	}

	return 1;
}

void SquareLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "Dx=" ) Dx.Input( fin );
	if ( var == "Dy=" ) Dy.Input( fin );
	Light::Input( var , fin );
}


double SquareLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {//面光源
	double shade = 0;
	//NEED TO IMPLEMENT
	int xLightNumber = 2;
	int yLightNumber = 2;
	Vector3 light;
	Vector3 V;
	bool flag = true;
	for (int inx = 0; inx < xLightNumber; inx++)
	{
		for (int iny = 0; iny < yLightNumber; iny++)
		{
			light = O + inx * Dx / ( xLightNumber * 1.0 ) - Dx / 2 + iny * Dy / (yLightNumber * 1.0) - Dy / 2;//分化面光源成单个点光源
			V = light - C;
			double dist = V.Module();
			flag = true;
			for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
			{
				CollidePrimitive tmp = now->Collide(C, V);
				if (EPS < (dist - tmp.dist)) {
					flag = false;
					break;
				}
			}
			if (flag) {
				shade += 1.0 / xLightNumber / yLightNumber;
			}
		}
	}
	return shade;
}

Primitive* SquareLight::CreateLightPrimitive()
{
	PlaneAreaLightPrimitive* res = new PlaneAreaLightPrimitive(O, Dx, Dy, color);
	lightPrimitive = res;
	return res;
}



void SphereLight::Input( std::string var , std::stringstream& fin ) {
	if ( var == "O=" ) O.Input( fin );
	if ( var == "R=" ) fin>>R;
	Light::Input( var , fin );
}


double SphereLight::CalnShade( Vector3 C , Primitive* primitive_head , int shade_quality ) {//球形光源
	double shade = 0;
	//NEED TO IMPLEMENT
	int w = 2;   //球可以分为多少个部分以及每个部分有多少光源
	int j = 2;

	int  num = (w - 1)*j + 2;
	float deltaW = PI / w;
	float deltaJ = 2 * PI / j;
	Vector3 light;
	Vector3 V;
	bool flag = true;
	for (int ww = 1; ww < w; ww++) {
		for (int jj = 0; jj < j; jj++) {
			float a = ww * deltaW;
			float b = jj * deltaJ;
			float x = sin(a)*cos(b);
			float y = sin(a)*sin(b);
			float z = cos(a);
			
			light.x = x * R;
			light.y = y * R;
			light.z = z * R;

			light += O;
			V = light - C;
			double dist = V.Module();
			flag = true;
			for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
			{
				CollidePrimitive tmp = now->Collide(C, V);
				if (EPS < (dist - tmp.dist)) {
					flag = false;
					break;
				}
			}
			if (flag) {
				shade += 1.0 / num;
			}
		}
	}
	//两个极点的光源
	light = O + Vector3(0. ,0., R);//顶部光源点光源
	V = light - C;
	double dist = V.Module();
	for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
	{
		CollidePrimitive tmp = now->Collide(C, V);
		if (EPS < (dist - tmp.dist)) {
			flag = false;
			break;
		}
	}
	if (flag) {
		shade += 1.0 / num;
	}

	light = O + Vector3(0., 0., -R);//底部光源
	V = light - C;
	dist = V.Module();
	for (Primitive* now = primitive_head; now != NULL; now = now->GetNext())
	{
		CollidePrimitive tmp = now->Collide(C, V);
		if (EPS < (dist - tmp.dist)) {
			flag = false;
			break;
		}
	}
	if (flag) {
		shade += 1.0 / num;
	}
	return shade;
}


Primitive* SphereLight::CreateLightPrimitive()
{
	SphereLightPrimitive* res = new SphereLightPrimitive(O, R, color);
	lightPrimitive = res;
	return res;
}

