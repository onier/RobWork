#include "RenderContacts.hpp"

#include <boost/foreach.hpp>

using namespace rw::sensor;
using namespace rw::math;

RenderContacts::RenderContacts():
	_quadratic(NULL)
{
	_quadratic = gluNewQuadric();
}

RenderContacts::~RenderContacts()
{
	gluDeleteQuadric(_quadratic);
	//delete _quadratic;
}

void RenderContacts::addContact(const Contact3D& contacts){
	_contacts.push_back(contacts);
}

void RenderContacts::addContacts(const std::vector<Contact3D>& contacts){
	BOOST_FOREACH(const Contact3D& contact, contacts){
		_contacts.push_back(contact);
	}
}

void RenderContacts::setContacts(const std::vector<Contact3D>& contacts){
	_contacts = contacts;
}

void RenderContacts::draw(DrawType type, double alpha) const{
    BOOST_FOREACH(const Contact3D& con, _contacts){
    	Vector3D<> pos = con.p;
    	Vector3D<> nforce = con.n * con.normalForce;
		//if( _force.norm2()<0.001 )
		//    return;

		glPushMatrix();

		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(pos(0),pos(1),pos(2));// Center The Cone
		gluSphere( _quadratic, 0.001, 32, 32);    // Draw Our Sphere

		glBegin(GL_LINES);
		 glColor3f(1.0, 0.0, 0.0);
		 glVertex3d(0,0,0);
		 glVertex3d(nforce(0),nforce(1),nforce(2));
		glEnd();

		glPopMatrix();

	  	double size = con.normalForce;
		glPushMatrix();
		glColor3f(1.0, 0.0, 0.0);
		glTranslatef(pos(0),pos(1),pos(2));// Center The Cone
		gluCylinder(
			_quadratic,
			0.001,
			size*con.mu,
			size,
			32,
			32);    // Draw Our Cylinder

		glPopMatrix();
		glPushMatrix();
		glColor3f(0.0, 0.0, 1.0);
		glTranslatef(pos(0),pos(1),pos(2));// Center The Cone
		gluCylinder(
			_quadratic,
			0.001,
			(float)con.normalForce,
			size,
			32,
			32);    // Draw Our Cylinder

		glPopMatrix();
    }

}
