#include "UserObject.h"
#include "VulkanApplication.h"

Polyline::Polyline()
	: device(VulkanApplication::GetInstance()->device)
{
	obj = new Object();
}

Polyline::~Polyline() 
{
	
}

void Polyline::create(std::vector<double> K, unsigned int order, float step, float length, float thickness) 
{
	this->K = K;
	this->order = order;
	this->step = step;
	this->length = length;
	this->thickness = thickness;
	createVertices();

	obj->create(OBJECT_FLAG_USE_3D, NONE_TEXTURE);
}

void Polyline::destroy() 
{
	obj->destroy();
}

void Polyline::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		obj->setBindingFlag(index, false);
		return;
	}
	obj->setBindingFlag(index, true);
	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers[] = {obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
	vkCmdBindIndexBuffer(commandBuffer, obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipelineLayout(), 0, 1, &((obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, obj->getIndicesSize(), 1, 0, 0, 0);
}

void Polyline::applyState() 
{ 
	if (obj->getState() == OBJECT_STATE_UPDATE) {
		for (int i=0; i<oldBuffer.size(); i++) {
			vkDestroyBuffer(device, oldBuffer[i], nullptr);
		    vkFreeMemory(device, oldBufferMemory[i], nullptr);
		}
		oldBuffer.clear();
		oldBufferMemory.clear();
		obj->setState(OBJECT_STATE_DRAWABLE);
		return ;
	}
	obj->applyState(); 
}

void Polyline::updateVertices(std::vector<double> K, unsigned int order, float step, float length, float thickness) 
{
	this->K = K;
	this->order = order;
	this->step = step;
	this->length = length;
	this->thickness = thickness;
	
	createVertices();

	oldBuffer.push_back(obj->getVertexBuffer());
	oldBufferMemory.push_back(obj->getVertexBufferMemory());
	oldBuffer.push_back(obj->getIndexBuffer());
	oldBufferMemory.push_back(obj->getIndexBufferMemory());
	obj->setState(OBJECT_STATE_UPDATE);

	obj->createVertexBuffers();
    obj->createIndexBuffer();
}

void Polyline::updateVertices(std::vector<double> K, unsigned int order) 
{
	updateVertices(K, order, step, length, thickness);
}

float Polyline::getLen(float y)
{
	if(K[2] == 0){
		float x = getX(y);
		return sqrtf(x*x + y*y);	
	}
	float k = K[1]+2*K[2]*y;
	float r1 = ((k)*sqrt(1 + k*k) + asinh(k)) / (4*K[2]);
	float r2 = (K[1]*sqrt(1 + K[1]*K[1]) + asinh(K[1])) / (4*K[2]);
	return r1 - r2;
}

float Polyline::getY(float dist)
{
	float y = dist;
	float step = 0.1;
	float res = getLen(y);
	while(true){
		float n;
		if(res > dist){
			n = getLen(y-step);
			if(res - dist < abs(n - dist)) return y;
			y -= step;
		} else {
			n = getLen(y+step);
			if(dist - res < abs(n - dist)) return y;
			y += step;
		}
	}
}

float Polyline::getX(float y)
{
	return K[2]*y*y + K[1]*y + K[0];
}

float Polyline::getTangent(float y)
{
	return 2*K[2]*y + K[1];
}

void Polyline::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;
	model = glm::identity<glm::mat4>();
	view = glm::lookAt (Camera::getInstance().eye,
						Camera::getInstance().eye + Camera::getInstance().eye2center,
						Camera::getInstance().up);

	proj = glm::perspective(glm::radians(60.0f), 		            //fov-y
							WINDOW_WIDTH / (float) WINDOW_HEIGHT,	//aspect 
							0.001f,						            //z near
							100.0f);					            //z far
	proj[1][1] *= -1;

	obj->updateUniformBuffer(currentImage, proj * view * model);
}

void Polyline::createVertices()
{
	std::vector<float> vertices;
	std::vector<uint16_t> indices;
	float x=0, y=0;
	std::vector<glm::vec2> values;
	for(unsigned int i=0; i<order; i++) x += K[i] * pow(y, i);
	values.push_back({x, y});
	bool rl = false;
	while (getLen(y) < length){
		x = 0;
		y += step;
		for(unsigned int i=0; i<=order; i++) x += K[i] * pow(y, i);
		
		glm::vec2 w = {-(y-values.back().y), x-values.back().x};
		glm::vec2 p;
		if(vertices.size()==0){
			p = values.back() + (w / (glm::length(w) / (thickness/2)));
			vertices.push_back(p.x);
			vertices.push_back(p.y);
			vertices.push_back(0);
		}

		if (rl) p = values.back() + (w / (glm::length(w) / (thickness/2)));
		else p = values.back() - (w / (glm::length(w) / (thickness/2)));
		rl = !rl;
		
		vertices.push_back(p.x);
		vertices.push_back(p.y);
		vertices.push_back(0);

		int count = vertices.size()/3;
		if(count >= 3){
			if(rl) {
				indices.push_back(count-2);
				indices.push_back(count-3);
				indices.push_back(count-1);
			} else {
				indices.push_back(count-3);
				indices.push_back(count-2);
				indices.push_back(count-1);
			}
		}
		
		values.push_back({x, y});
		// p_len += glm::length(w);
	}
	VertexLayout vertexLayout;
	vertexLayout.count = vertices.size()/3;
	vertexLayout.push(sizeof(float), 3);

	obj->setVertices(vertices);
	obj->setIndices(indices);
	obj->setVertexLayout(vertexLayout);
}

//////////////////////////////////////////////////////////////

Notification::Notification()
{
	icon_obj = new Object();
	msg_obj= new Text();
}

Notification::~Notification()
{

}

void Notification::create(std::string texturePath, std::string msg, float duration, bool call) 
{
	int size = 50;
	icon_obj->setWidth({size, size});
	icon_obj->create(OBJECT_FLAG_USE_TEXTURE, texturePath);

	msg_obj->setWidth({0, size-5});
	msg_obj->create(OBJECT_FLAG_USE_TEXTURE, FONT_PATH_NANUMGOTHIC_EB, msg);
	msg_obj->setColor({1, 1, 1, 1});

	icon_obj->setPosition({size/2, WINDOW_HEIGHT-size*3/2, 0});
	msg_obj->setPosition({icon_obj->getPosition().x + icon_obj->getWidth().x + 10, icon_obj->getPosition().y + 2.5, 0});

	this->duration = duration;
	createdTime = ros::Time::now();

	this->call = call;
}

void Notification::destroy() 
{
	icon_obj->destroy();
	msg_obj->destroy();
}

void Notification::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(icon_obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		icon_obj->setBindingFlag(index, false);
		msg_obj->setBindingFlag(index, false);
		return;
	}
	icon_obj->setBindingFlag(index, true);
	msg_obj->setBindingFlag(index, true);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, icon_obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers1[] = {icon_obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, icon_obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, icon_obj->getPipeline()->getPipelineLayout(), 0, 1, &((icon_obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, icon_obj->getIndicesSize(), 1, 0, 0, 0);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, msg_obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers2[] = {msg_obj->getVertexBuffer()};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers2, offsets);
	vkCmdBindIndexBuffer(commandBuffer, msg_obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, msg_obj->getPipeline()->getPipelineLayout(), 0, 1, &((msg_obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, msg_obj->getIndicesSize(), 1, 0, 0, 0);
}

void Notification::applyState() 
{
	if((ros::Time::now()-createdTime).sec >= duration && getState() == OBJECT_STATE_DRAWABLE) {
		icon_obj->setState(OBJECT_STATE_RESERVE_DELETE); 
		msg_obj->setState(OBJECT_STATE_RESERVE_DELETE); 
	}
	icon_obj->applyState(); 
	msg_obj->applyState(); 
	// if(getState() == OBJECT_STATE_DELETABLE){
	// 	delete icon_obj;
	// 	delete msg_obj;
	// }
}

void Notification::moveUp()
{
	int size = 50;
	// std::cout<<icon_obj->getPosition().x<<" "<<icon_obj->getPosition().y<<" "<<icon_obj->getPosition().z<<"\n";
	icon_obj->setPosition(icon_obj->getPosition() + glm::vec3({0, size*3/2, 0}));
	msg_obj->setPosition(msg_obj->getPosition() + glm::vec3({0, size*3/2, 0}));
}

void Notification::moveDown()
{
	int size = 50;
	// std::cout<<icon_obj->getPosition().x<<" "<<icon_obj->getPosition().y<<" "<<icon_obj->getPosition().z<<"\n";
	icon_obj->setPosition(icon_obj->getPosition() + glm::vec3({0, -size*3/2, 0}));
	msg_obj->setPosition(msg_obj->getPosition() + glm::vec3({0, -size*3/2, 0}));
}

void Notification::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), icon_obj->getPosition()) *
			glm::scale(glm::mat4(1.0f), {icon_obj->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	icon_obj->updateUniformBuffer(currentImage, proj * view * model);

	// glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), msg_obj->getPosition()) *
			glm::scale(glm::mat4(1.0f), {msg_obj->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	msg_obj->updateUniformBuffer(currentImage, proj * view * model);
}

////////////////////

void NotificationManager::addKakao(std::string msg) 
{
	for(int i=0; i<notis.size(); i++){
		if(notis[i]->isCall()) continue;
		if(notis.size() > 4){
			notis[i]->setState(OBJECT_STATE_RESERVE_DELETE);
			notis[i] = nullptr;
			notis.erase(notis.begin() + i);
			i--;
		} else {
			notis[i]->moveDown();
		}
	}
	Notification *noti = new Notification();
	noti->create("../../../src/vulkan/textures/kakao_.png", msg, 5);
	if(hasCall) noti->moveDown();
	VulkanApplication::GetInstance()->objects.push_back(noti);
	notis.push_back(noti);
}

void NotificationManager::addSMS(std::string msg) 
{
	for(int i=0; i<notis.size(); i++){
		if(notis[i]->isCall()) continue;
		if(notis.size() > 4){
			notis[i]->setState(OBJECT_STATE_RESERVE_DELETE);
			notis[i] = nullptr;
			notis.erase(notis.begin() + i);
			i--;
		} else {
			notis[i]->moveDown();
		}
	}
	Notification *noti = new Notification();
	noti->create("../../../src/vulkan/textures/msgg.png", msg, 5);
	if(hasCall) noti->moveDown();
	VulkanApplication::GetInstance()->objects.push_back(noti);
	notis.push_back(noti);
}

void NotificationManager::addCall(std::string name, int type) 
{
	if(hasCall){
		if(type == 2) {
			for(int i=0; i<notis.size(); i++){
				if(notis[i]->isCall()){
					notis[i]->setState(OBJECT_STATE_RESERVE_DELETE);
					notis[i] = nullptr;
					notis.erase(notis.begin() + i);
					i--;
					hasCall = false;
				} else {
					notis[i]->moveUp();
				}
			}
		} else if (type == 0){
			for(int i=0; i<notis.size(); i++){
				if(notis[i]->isCall()){
					notis[i]->setColor({1, 0, 0, 1});
					notis[i]->setDuration(0.5);
					break;
				}
			}
		}
		return ;
	}
	if(type == 1){
		for(int i=0; i<notis.size(); i++){
			if(notis[i]->isCall()) continue;
			if(notis.size() > 4){
				notis[i]->setState(OBJECT_STATE_RESERVE_DELETE);
				notis[i] = nullptr;
				notis.erase(notis.begin() + i);
				i--;
			} else {
				notis[i]->moveDown();
			}
		}
		Notification *noti = new Notification();
		noti->create("../../../src/vulkan/textures/call_.png", name, 100, true);
		noti->setColor({0, 1, 0, 1});
		VulkanApplication::GetInstance()->objects.push_back(noti);
		notis.push_back(noti);
		hasCall = true;
	}
}

void NotificationManager::update()
{
	for(int i=0; i<notis.size(); i++){
		if(notis[i]->getState() == OBJECT_STATE_RESERVE_DELETE) {
			if(notis[i]->isCall()) {
				hasCall = false;
				for(auto noti : notis) noti->moveUp();
			}
			notis[i] = nullptr;
			notis.erase(notis.begin() + i);
			i--;
		}
	}
}

///////////////////////////////////////////////////////////////

Speed::Speed()
{
	speed = new Text;
	km = new Text;
}

void Speed::create(float value) 
{
	this->value = value;
	int size = 100;
	speed->setHeight(size);
	speed->create(0, FONT_PATH_ANVYL, std::to_string((int)(value*3.6)));
	speed->setColor({0, 1, 0, 1});
	
	km->setHeight(size*3/5);
	km->create(0, FONT_PATH_NANUMGOTHIC, " km/h");
	km->setColor({0, 1, 0, 1});
	
	speed->setPosition({WINDOW_WIDTH/2 - (speed->getWidth().x + km->getWidth().x)/2 - 30, 20 + speed->getWidth().y/2, 0});
	km->setPosition({speed->getPosition().x + speed->getWidth().x, speed->getPosition().y, 0});
}

void Speed::destroy() 
{
	speed->destroy();
	km->destroy();
}

void Speed::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(speed->getState() == OBJECT_STATE_RESERVE_DELETE){
		speed->setBindingFlag(index, false);
		km->setBindingFlag(index, false);
		return;
	}
	speed->setBindingFlag(index, true);
	km->setBindingFlag(index, true);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, speed->getPipeline()->getPipeline());

	VkBuffer vertexBuffers1[] = {speed->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, speed->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, speed->getPipeline()->getPipelineLayout(), 0, 1, &((speed->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, speed->getIndicesSize(), 1, 0, 0, 0);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, km->getPipeline()->getPipeline());

	VkBuffer vertexBuffers2[] = {km->getVertexBuffer()};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers2, offsets);
	vkCmdBindIndexBuffer(commandBuffer, km->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, km->getPipeline()->getPipelineLayout(), 0, 1, &((km->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, km->getIndicesSize(), 1, 0, 0, 0);
}

void Speed::applyState() 
{
	speed->applyState(); 
	km->applyState(); 
}

void Speed::updateSpeed(float value)
{
	this->value = value;
	speed->updateText(std::to_string((int)(value*3.6)));
	speed->setPosition({WINDOW_WIDTH/2 - (speed->getWidth().x + km->getWidth().x)/2, 20 + speed->getWidth().y/2, 0});
	km->setPosition({speed->getPosition().x + speed->getWidth().x, speed->getPosition().y, 0});
}

void Speed::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), speed->getPosition()) *
			glm::scale(glm::mat4(1.0f), {speed->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	speed->updateUniformBuffer(currentImage, proj * view * model);

	// glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), km->getPosition()) *
			glm::scale(glm::mat4(1.0f), {km->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	km->updateUniformBuffer(currentImage, proj * view * model);
}

//////////////////////////////////////////////////////////////////////////

Direction2D::Direction2D(){
	obj = new Object;
	dist = new Text;
}

void Direction2D::create(int16_t type, float distance) 
{
	this->type = type;
	std::string texturePath = getTexturePath();
	// std::string texturePath = "../../../src/vulkan/textures/turn_right.png";
	// std::string texturePath = "../../../src/vulkan/textures/turn_u.png";

	int size = 100;
	obj->setWidth({size, size});
	obj->create(OBJECT_FLAG_USE_TEXTURE, texturePath);
	
	dist->setHeight(size/2);
	dist->create(0, FONT_PATH_NANUMGOTHIC, (distance < 1000) ? std::to_string((int)distance) + " m" : 
															   std::to_string(distance/1000).substr(0, 4) + " km");
	// dist->setColor({0, 1, 0, 1});
	
	// obj->setPosition({WINDOW_WIDTH/2 - (speed->getWidth().x + km->getWidth().x)/2, 20 + speed->getWidth().y/2, 0});
	// km->setPosition({speed->getPosition().x + speed->getWidth().x, speed->getPosition().y, 0});
}

void Direction2D::destroy() 
{
	obj->destroy();
	dist->destroy();
}

void Direction2D::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		obj->setBindingFlag(index, false);
		dist->setBindingFlag(index, false);
		return;
	}
	obj->setBindingFlag(index, true);
	dist->setBindingFlag(index, true);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers1[] = {obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipelineLayout(), 0, 1, &((obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, obj->getIndicesSize(), 1, 0, 0, 0);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, dist->getPipeline()->getPipeline());

	VkBuffer vertexBuffers2[] = {dist->getVertexBuffer()};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers2, offsets);
	vkCmdBindIndexBuffer(commandBuffer, dist->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, dist->getPipeline()->getPipelineLayout(), 0, 1, &((dist->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, dist->getIndicesSize(), 1, 0, 0, 0);
}

void Direction2D::applyState() 
{
	obj->applyState(); 
	dist->applyState(); 
}

void Direction2D::setSize(float size)
{
	obj->setWidth({size, size});
	dist->setHeight(size/2);
	setPosition({obj->getPosition().x, obj->getPosition().y});
}

void Direction2D::setPosition(glm::vec2 position)
{
	dist->setPosition({position.x + obj->getWidth().x/2 - dist->getWidth().x/2, position.y, 0});
	obj->setPosition({position.x, dist->getPosition().y + dist->getWidth().y + 10, 0});
}

void Direction2D::updateType(int16_t type)
{	
	this->type = type;
	std::string texturePath = getTexturePath();

	obj->updateTexture(texturePath);
}

void Direction2D::setDistance(float distance)
{
	dist->updateText((distance < 1000) ? std::to_string((int)distance) + " m" : 
										 std::to_string(distance/1000).substr(0, 4) + " km");	
	dist->setPosition({obj->getPosition().x + obj->getWidth().x/2 - dist->getWidth().x/2, dist->getPosition().y, 0});
}

void Direction2D::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), obj->getPosition()) *
			glm::scale(glm::mat4(1.0f), {obj->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	obj->updateUniformBuffer(currentImage, proj * view * model);

	// glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), dist->getPosition()) *
			glm::scale(glm::mat4(1.0f), {dist->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	dist->updateUniformBuffer(currentImage, proj * view * model);
}

inline std::string Direction2D::getTexturePath()
{
	switch (type){
	case 11:		// 직진
		return "../../../src/vulkan/textures/turn/turn_11.png";
	case 12:		// 좌회전
		return "../../../src/vulkan/textures/turn/turn_12.png";
	case 13:		// 우회전
		return "../../../src/vulkan/textures/turn/turn_13.png";
	case 14:		// U 턴
		return "../../../src/vulkan/textures/turn/turn_14.png";
	case 16:		// 8시 방향 좌회전
		return "../../../src/vulkan/textures/turn/turn_16.png";
	case 17:		// 10시 방향 좌회전
		return "../../../src/vulkan/textures/turn/turn_17.png";
	case 18:		// 2시 방향 우회전
		return "../../../src/vulkan/textures/turn/turn_18.png";
	case 19:		// 4시 방향 우회전
		return "../../../src/vulkan/textures/turn/turn_19.png";
	case 117:		// 오른쪽 방향
		return "../../../src/vulkan/textures/turn/turn_117.png";
	case 118:		// 왼쪽 방향
		return "../../../src/vulkan/textures/turn/turn_118.png";
	case 119:		// 지하차도
		return "../../../src/vulkan/textures/turn/turn_119.png";
	case 120:		// 고가도로
		return "../../../src/vulkan/textures/turn/turn_120.png";
	case 121:		// 터널
		return "../../../src/vulkan/textures/turn/turn_121.png";
	case 123:		// 지하차도 옆
		return "../../../src/vulkan/textures/turn/turn_123.png";
	case 124:		// 고가도로 옆
		return "../../../src/vulkan/textures/turn/turn_124.png";
	// case 200:		// 출발지
	// 	return "../../../src/vulkan/textures/turn/qm.png";
	// case 201:		// 도착지
	// 	return "../../../src/vulkan/textures/turn/qm.png";
	default:		// others
		return "../../../src/vulkan/textures/qm.png";
	}
}

/////

Direction3D::Direction3D(){
	obj = new Object;
}

void Direction3D::create(int16_t type, float distance) 
{
	this->type = type;
	std::string texturePath = getTexturePath();

	obj->setWidth({2, 2});
	setDistance(distance);
	obj->create(OBJECT_FLAG_USE_TEXTURE | OBJECT_FLAG_USE_3D, texturePath);
	
	// obj->setPosition({WINDOW_WIDTH/2 - (speed->getWidth().x + km->getWidth().x)/2, 20 + speed->getWidth().y/2, 0});
}

void Direction3D::destroy() 
{
	obj->destroy();
}

void Direction3D::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		obj->setBindingFlag(index, false);
		return;
	}
	obj->setBindingFlag(index, true);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers1[] = {obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipelineLayout(), 0, 1, &((obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, obj->getIndicesSize(), 1, 0, 0, 0);
}

void Direction3D::applyState() 
{
	obj->applyState(); 
}

void Direction3D::setDistance(float distance)
{
	float x, y, dx;
	y = VulkanApplication::GetInstance()->lane->getY(distance);
	x = VulkanApplication::GetInstance()->lane->getX(y);
	dx = VulkanApplication::GetInstance()->lane->getTangent(y);
	obj->setPosition({x, y, 1});
	obj->setNormal({-dx, -1, 0});
}

void Direction3D::setPosition(glm::vec2 position)
{
	// obj->setNormal({0, -1, 0});
}

void Direction3D::updateType(int16_t type)
{
	this->type = type;
	std::string texturePath = getTexturePath();

	obj->updateTexture(texturePath);
}

void Direction3D::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	glm::mat4 r;
	glm::vec3 normal = obj->getNormal();
	if(normal.x == 0 && normal.y == 0) r = glm::mat4(1.0f);
	else {
		r = glm::rotate(glm::mat4(1.0f),
						(float)-atan2(sqrtf(normal.x*normal.x+normal.y*normal.y), normal.z), 
						glm::vec3({normal.y, -normal.x, 0})) *
			glm::rotate(glm::mat4(1.0f),
						(float)atan2(normal.x, -normal.y), 
						glm::vec3({0, 0, 1}));
	}

	model = glm::translate(glm::mat4(1.0f), obj->getPosition()) *					
			r *
			glm::translate(glm::mat4(1.0f), {-obj->getWidth().x/2, -obj->getWidth().y/2, 0}) *
			glm::scale(glm::mat4(1.0f), {obj->getWidth(), 1});
	// model = glm::identity<glm::mat4>();

	view = glm::lookAt (Camera::getInstance().eye,
						Camera::getInstance().eye + Camera::getInstance().eye2center,
						Camera::getInstance().up);

	proj = glm::perspective(glm::radians(60.0f), 		            //fov-y
							WINDOW_WIDTH / (float) WINDOW_HEIGHT,	//aspect 
							0.001f,						            //z near
							500.0f);					            //z far
	proj[1][1] *= -1;
	obj->updateUniformBuffer(currentImage, proj * view * model);
}

inline std::string Direction3D::getTexturePath()
{
	switch(type) {
	case 12:		// 좌회전
		return "../../../src/vulkan/textures/direction_left.png";
	case 13:		// 우회전
		return "../../../src/vulkan/textures/direction_right.png";
	case 14:		// U 턴
		return "../../../src/vulkan/textures/direction_u.png";
	case 17:		// 10시 방향 좌회전
		return "../../../src/vulkan/textures/direction_left.png";
	case 118:		// 왼쪽 방향
		return "../../../src/vulkan/textures/direction_left.png";
	case 119:		// 지하차도
		return "../../../src/vulkan/textures/qm.png";
	case 120:		// 고가도로
		return "../../../src/vulkan/textures/qm.png";
	case 200:		// 출발지
		return "../../../src/vulkan/textures/qm.png";
	case 201:		// 도착지
		return "../../../src/vulkan/textures/qm.png";
	default:		// others
		return "../../../src/vulkan/textures/qm.png";
	}
}

////

Navigation::Navigation()
{
	planeData.resize(2);
	planeData[0] = nullptr;
	planeData[1] = nullptr;

	placeData = nullptr;
}

void Navigation::updateData(int16_t type, float distance, int16_t next_type, float next_distance)
{
	this->type = type;
	this->nextDistance = distance;

	if (planeData[0] == nullptr) {
		nowDistance = distance;
		planeData[0] = new Direction2D;
		planeData[0]->create(type, distance);
		planeData[0]->setPosition({WINDOW_WIDTH/2 + 150, 50});
		VulkanApplication::GetInstance()->objects.push_back(planeData[0]);
	} else {
		if(planeData[0]->getType() != type) {
			nowDistance = distance;
			planeData[0]->updateType(type);
		}
		planeData[0]->setDistance(distance);
		// planeData[0]->updateDistance(distance);
	}

	// if (placeData == nullptr && distance <= 100) {
	// 	placeData = new Direction3D;
	// 	placeData->create(type, distance);
	// 	VulkanApplication::GetInstance()->objects.push_back(placeData);
	// } else if (placeData != nullptr && distance <= 100) {
	// 	if(placeData->getType() != type) {
	// 		placeData->updateType(type);
	// 		placeData->setDistance(distance);
	// 	}
	// 	// placeData->setDistance(distance);
	// } else if (placeData != nullptr && distance > 100) {
	// 	placeData->setState(OBJECT_STATE_RESERVE_DELETE);
	// 	placeData = nullptr;
	// }

	if (planeData[1] == nullptr && next_type != -1) {
		planeData[1] = new Direction2D;
		planeData[1]->create(next_type, next_distance);
		planeData[1]->setSize(70);
		planeData[1]->setPosition({WINDOW_WIDTH/2 + 330, 50});
		VulkanApplication::GetInstance()->objects.push_back(planeData[1]);
	} else if (planeData[1] != nullptr && next_type != -1) {
		if(planeData[1]->getType() != type) planeData[1]->updateType(next_type);
		planeData[1]->setDistance(next_distance);
	} else if (planeData[1] != nullptr && next_type == -1) {
		planeData[1]->setState(OBJECT_STATE_RESERVE_DELETE);
		planeData[1] = nullptr;
	}
}

void Navigation::update()
{	
	float std_speed = VulkanApplication::GetInstance()->speed->getSpeed() / (WINDOW_FPS/4);
	float velocity = std_speed + (nowDistance - nextDistance) / WINDOW_FPS;
	// std::cout<<VulkanApplication::GetInstance()->speed->getSpeed()<<" "<<std_speed<<"\n";
	// std::cout<<nextDistance <<" "<<nowDistance<<"\n";
	int margin = 0, target_distance = 100, near = 7;
	if (placeData == nullptr && nowDistance <= target_distance + margin && nowDistance >= near + margin) {
		placeData = new Direction3D;
		placeData->create(type, nowDistance - margin);
		VulkanApplication::GetInstance()->objects.push_back(placeData);
	} else if (placeData != nullptr && nowDistance <= target_distance + margin && nowDistance >= near + margin) {
		if(placeData->getType() != type) {
			placeData->updateType(type);
		}
		placeData->setDistance(nowDistance - margin);
	} else if (placeData != nullptr && (nowDistance > target_distance + margin || nowDistance < near + margin)) {
		placeData->setState(OBJECT_STATE_RESERVE_DELETE);
		placeData = nullptr;
	}
	nowDistance -= velocity;
}

////////////////////////////////////////////////////////

Warning::Warning() {
	obj = new Object;
}

void Warning::create(double x, double y, double depth, uint8_t level) 
{	
	obj->setWidth({1, 1});
	setPosition(x, y, depth, level);
	obj->create(OBJECT_FLAG_USE_TEXTURE | OBJECT_FLAG_USE_3D, "../../../src/vulkan/textures/warning.png");
	
	duration = 1;
	createdTime = ros::Time::now();
	// obj->setPosition({WINDOW_WIDTH/2 - (speed->getWidth().x + km->getWidth().x)/2, 20 + speed->getWidth().y/2, 0});
}

void Warning::destroy() 
{
	obj->destroy();
}

void Warning::setPosition(double x, double y, double depth, uint8_t level)
{
	createdTime = ros::Time::now();
	if(depth < 3) {
		obj->setWidth({depth/3, depth/3});
	} else {
		obj->setWidth({1, 1});	
	}
	// std::cout<<x<<" "<<depth<<" "<<y<<"\n";
	obj->setPosition({x, depth, y});
	obj->setNormal({0, -1, 0});
	if (level == 1) {
		obj->setColor({1, 1, 0, 1});
	} else if (level == 2) {
		obj->setColor({1, 0.25, 0, 1});
	} else if (level == 3) {
		obj->setColor({1, 0, 0, 1});
	}
}

void Warning::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		obj->setBindingFlag(index, false);
		return;
	}
	obj->setBindingFlag(index, true);

	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipeline());

	VkBuffer vertexBuffers1[] = {obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipelineLayout(), 0, 1, &((obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, obj->getIndicesSize(), 1, 0, 0, 0);
}

void Warning::applyState() 
{
	if(obj->getState() == OBJECT_STATE_DELETABLE) return;
	if((ros::Time::now()-createdTime).sec >= duration && getState() == OBJECT_STATE_DRAWABLE) {
		obj->setState(OBJECT_STATE_RESERVE_DELETE);
	}
	obj->applyState(); 
}

void Warning::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	glm::mat4 r;
	glm::vec3 normal = obj->getNormal();
	if(normal.x == 0 && normal.y == 0) r = glm::mat4(1.0f);
	else {
		r = glm::rotate(glm::mat4(1.0f),
						(float)-atan2(sqrtf(normal.x*normal.x+normal.y*normal.y), normal.z), 
						glm::vec3({normal.y, -normal.x, 0})) *
			glm::rotate(glm::mat4(1.0f),
						(float)atan2(normal.x, -normal.y), 
						glm::vec3({0, 0, 1}));
	}

	model = glm::translate(glm::mat4(1.0f), obj->getPosition()) *					
			r *
			glm::translate(glm::mat4(1.0f), {-obj->getWidth().x/2, -obj->getWidth().y/2, 0}) *
			glm::scale(glm::mat4(1.0f), {obj->getWidth(), 1});
	// model = glm::identity<glm::mat4>();

	view = glm::lookAt (glm::vec3({0, 0, 0}),
						glm::vec3({0, 1, 0}),
						Camera::getInstance().up);

	proj = glm::perspective(glm::radians(52.2f), 		            //fov-y
							WINDOW_WIDTH / (float) WINDOW_HEIGHT,	//aspect 
							0.001f,						            //z near
							500.0f);					            //z far
	proj[1][1] *= -1;
	obj->updateUniformBuffer(currentImage, proj * view * model);
}











//////////////////////////////////////////////////////////////////

BackGround::BackGround(){
	obj = new Object;
}

void BackGround::create() 
{	
	obj->setWidth({1280, 720});
	obj->setPosition({0, 0, 10});
	obj->create(OBJECT_FLAG_USE_TEXTURE, "../../../src/vulkan/textures/qm.png");
}

void BackGround::destroy() 
{
	obj->destroy();
}

void BackGround::bindCommandBuffer(VkCommandBuffer commandBuffer, int index) 
{
	if(obj->getState() == OBJECT_STATE_RESERVE_DELETE){
		obj->setBindingFlag(index, false);
		return;
	}
	obj->setBindingFlag(index, true);
	
	vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipeline());
	
	VkBuffer vertexBuffers1[] = {obj->getVertexBuffer()};
	VkDeviceSize offsets[] = {0};
	vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers1, offsets);
	vkCmdBindIndexBuffer(commandBuffer, obj->getIndexBuffer(), 0, VK_INDEX_TYPE_UINT16);
	vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, obj->getPipeline()->getPipelineLayout(), 0, 1, &((obj->getDescriptor()->getDescriptorSets())[index]), 0, nullptr);
	vkCmdDrawIndexed(commandBuffer, obj->getIndicesSize(), 1, 0, 0, 0);
}

void BackGround::applyState() 
{
	obj->applyState(); 
}

void BackGround::updateTexture(std::vector<unsigned char> data)
{
	obj->updateTexture(data, 1280, 720);
}

void BackGround::updateUniformBuffer(uint32_t currentImage) 
{
	glm::mat4 model, view, proj;		
	model = glm::translate(glm::mat4(1.0f), obj->getPosition()) *
			glm::scale(glm::mat4(1.0f), {obj->getWidth(), 1});
	view = glm::lookAt (glm::vec3(0, 0, -1),		        //eye
						glm::vec3(0, 0, 0), 	            //center
						glm::vec3(0, -1, 0));	            //up axis
	proj = glm::ortho((float)0, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, (float)0, 0.0f, 11.0001f);
	proj[1][1] *= -1;
	obj->updateUniformBuffer(currentImage, proj * view * model);
}





