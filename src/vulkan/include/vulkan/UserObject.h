#ifndef __USEROBJECT_H__
#define __USEROBJECT_H__

#include "header.h"

#include "Object.h"

class UserObject
{
public:
	// virtual void create();
	virtual void destroy() = 0;

	virtual void bindCommandBuffer(VkCommandBuffer, int) = 0;
	
	virtual void setState(int) = 0;
	virtual inline int getState() const = 0;
	virtual void applyState() = 0;

	virtual void updateUniformBuffer(uint32_t) = 0;
};

class Polyline : public UserObject
{
private:
	VkDevice 			&device;

	std::vector<double> K;
	unsigned int 		order;
	float 				step;
	float 				length;
	float 				thickness;

	Object 						*obj;
	std::vector<VkBuffer> 		oldBuffer;
	std::vector<VkDeviceMemory> oldBufferMemory;
public:
	Polyline();
	~Polyline();

	void create(std::vector<double>, unsigned int, float, float, float);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { obj->setState(state); }
	inline int getState() const { return obj->getState(); }
	void applyState();

	void updateVertices(std::vector<double>, unsigned int, float, float, float);
	void updateVertices(std::vector<double>, unsigned int);
	float getLen(float);
	float getY(float);
	float getX(float);
	float getTangent(float);
	void updateUniformBuffer(uint32_t);

private:
	void createVertices();
};

class Notification : public UserObject
{
private:
	ros::Time 	createdTime;	
	float 		duration;
	bool 		call = false;

    Object  *icon_obj;
    Text    *msg_obj;
public:
	Notification();
	~Notification();

	void create(std::string, std::string, float, bool = false);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { icon_obj->setState(state); msg_obj->setState(state); }
	inline int getState() const { return icon_obj->getState(); }
	void applyState();

	void moveUp();
	void moveDown();
	void setColor(glm::vec4 color) { msg_obj->setColor(color); }
	bool isCall() const { return call; }
	void setNotCall() { call = false; }
	void setDuration(float duration) { createdTime = ros::Time::now(); this->duration = duration; }

	void updateUniformBuffer(uint32_t);
};

class NotificationManager 
{
private:
	std::vector<Notification *> notis;
	int hasCall = false;
public:
	void addSMS(std::string);
	void addKakao(std::string);
	void addCall(std::string, int);

	void update();
};

class Speed : public UserObject
{
private:
	float value;
    Text    *speed;
    Text    *km;
public:
	Speed();

	void create(float);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { speed->setState(state); km->setState(state); }
	inline int getState() const { return speed->getState(); }
	void applyState();

	float getSpeed() { return value; }
	void updateSpeed(float);
	void updateUniformBuffer(uint32_t);
};

class Direction2D : public UserObject
{
private:
	Object 	*obj;
	Text	*dist;
	int16_t type;
public:
	Direction2D();
	void create(int16_t, float);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { obj->setState(state); dist->setState(state); }
	inline int getState() const { return dist->getState(); }
	void applyState();

	void setSize(float);
	void setPosition(glm::vec2);
	void updateType(int16_t);
	inline int16_t getType() { return type; }
	void setDistance(float);
	void updateUniformBuffer(uint32_t);
	inline std::string getTexturePath();
};

class Direction3D : public UserObject
{
private:
	
	Object *obj;
	int		direction;		// 0 ~ 360
	int16_t type;
public:
	Direction3D();
	void create(int16_t, float);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { obj->setState(state); }
	inline int getState() const { return obj->getState(); }
	void applyState();

	void updateType(int16_t);
	inline int16_t getType() { return type; }
	void setDistance(float);
	void setPosition(glm::vec2);
	void updateUniformBuffer(uint32_t);
	inline std::string getTexturePath();
};

class Navigation
{
private:
	uint16_t type;
	float nextDistance;
	float nowDistance;

	Speed *speedData;
	std::vector<Direction2D *> planeData;
	Direction3D *placeData;
public:
	Navigation();
	
	void updateSpeed(float);
	void updateData(int16_t, float, int16_t, float);
	void update();
};

// 개수?????
class Warning : public UserObject
{
private:
	ros::Time 	createdTime;	
	float 		duration;
	Object *obj;
public:
	Warning();
	void create(double, double, double, uint8_t);
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { obj->setState(state); }
	inline int getState() const { return obj->getState(); }
	void applyState();

	void setPosition(double, double, double, uint8_t);
	void updateUniformBuffer(uint32_t);
};




class BackGround : public UserObject
{
private:
	Object *obj;
public:
	BackGround();
	void create();
	void destroy();

	void bindCommandBuffer(VkCommandBuffer, int);
	
	void setState(int state) { obj->setState(state); }
	inline int getState() const { return obj->getState(); }
	void applyState();

	void updateTexture(std::vector<unsigned char>);
	void updateUniformBuffer(uint32_t);	
};

#endif // __USEROBJECT_H__