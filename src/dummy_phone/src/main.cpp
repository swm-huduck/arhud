#include <iostream>
#include <ros/ros.h>

#include "dummy_phone/speed.h"
#include "dummy_phone/navigation_turn_event.h"
#include "dummy_phone/setting.h"
#include "dummy_phone/call.h"
#include "dummy_phone/sms.h"
#include "dummy_phone/kakao_talk.h"
#include "dummy_phone/eye_position.h"

/* 차선에만 라이트를 해서
전제조건
    이번 프로젝트에서는 악조건을 제외
    ex) 비오는날, 터널에서 밖으로 나갈 때 -> 앞으로 어떻게 하겠다.
    디폴트 값으로 하겠다 -> 굳이 언급할 필요 없을 듯?
    악조건은 앞으로 고도화 과정에서 해결하겠다.
    시간이 짧아서 못했다.
    교차로나 차선 끊긴 부분 -> 시간이 짧으니까 그냥 쭉 이어지도록, 지도 데이터 축적을 통해
        지도데이터는 어떻게? -> 서비스를 하면서 축적, API회사와의 협업을 통해 

하우징에 관한 의견
    김 멘토님 -> 굳이 기술을 보여줄 때 중요한 부분은 아닐 거 같다.
    ROI를 따져봤을 때 투자한 시간대비 결과가 나오는지
    고정을 꼭 어디에 해야된다면 -> 앵글 같은거로 고정만 해라 (청계천에서 잘 해줌 사이즈 맞는거 구해서 )     */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy");

	ros::NodeHandle nodeHandler;
    ros::Publisher callPublisher = nodeHandler.advertise<dummy_phone::call>("call", 1);
    ros::Publisher smsPublisher = nodeHandler.advertise<dummy_phone::sms>("sms", 1);
    ros::Publisher kakaoPublisher = nodeHandler.advertise<dummy_phone::kakao_talk>("kakao_talk", 1);
    ros::Publisher speedPublisher = nodeHandler.advertise<dummy_phone::speed>("speed", 1);
    ros::Publisher navigationPublisher = nodeHandler.advertise<dummy_phone::navigation_turn_event>("navigation_turn_event", 1);
    
	srand((unsigned int)time(NULL));
	int ev[] = {11, 12, 13, 14, 16, 17, 18, 19, 117, 118, 119, 120, 121, 123, 124, 200, 201};
	std::vector<std::string> name_ex = {"이정우", "박유천", "정용훈", "엄마", "아빠", "John", "Jake"};
	std::vector<std::string> msg_ex = {"안녕!", "뭐해??", "밥 뭐먹을까", "어제 말했던거 ..."};
	
	int cnt=0;
    while(ros::ok()){
		int choosePub;
		std::cout<<"choose msg <0 : call> <1 : sms> <2 : kakao> <3 : Navi> <4 : Navi(next no event)>\n : ";
		std::cin>>choosePub;
		if(choosePub == 0) {			//call
			dummy_phone::call callData;
			
			std::cout<<"type name\n : ";
			// std::cin>>callData.name;
			callData.name = name_ex[rand()%name_ex.size()];; ;

			std::cout<<"choose call type <0 : end> <1 : 전화 옴> <2 : 전화 받음>\n : ";
			std::cin>>callData.call_type;
			callData.call_type -= '0'; 
			callPublisher.publish(callData);
			ros::spinOnce();
		} else if (choosePub == 1) {		//sms
			dummy_phone::sms smsData;
			
			std::cout<<"type name\n : ";
			// std::cin>>smsData.name;
			smsData.name = name_ex[rand()%name_ex.size()];

			std::cout<<"type content\n : ";
			// std::cin>>smsData.content; 
			smsData.content = msg_ex[rand()%msg_ex.size()];; 
			smsPublisher.publish(smsData);
			ros::spinOnce();
		} else if (choosePub == 2) {		//kakao
			dummy_phone::kakao_talk kakaoData;
			
			std::cout<<"type name\n : ";
			// std::cin>>kakaoData.name;
			kakaoData.name = name_ex[rand()%name_ex.size()];

			std::cout<<"type content\n : ";
			// std::cin>>kakaoData.content;  
			kakaoData.content = msg_ex[rand()%msg_ex.size()];; 
			kakaoPublisher.publish(kakaoData);
			ros::spinOnce();
		} else if (choosePub == 3) {		//navi
			dummy_phone::navigation_turn_event naviData;
			dummy_phone::speed spd;

			float speed = rand()%100;
			spd.value = speed;
			speedPublisher.publish(spd);

			std::cout<<"type name\n : ";
			// std::cin>>kakaoData.name;
			naviData.next_turn_type = ev[rand()%17];
			naviData.next_left_distance = rand()%2000 + 1;

			naviData.next_next_turn_type = ev[rand()%15];
			naviData.next_next_left_distance = rand()%2000 + 1;
			navigationPublisher.publish(naviData);
			ros::spinOnce();
		} else if (choosePub == 4) {		//kakao
			dummy_phone::navigation_turn_event naviData;
			
			std::cout<<"type name\n : ";
			// std::cin>>kakaoData.name;
			naviData.next_turn_type = 0;
			naviData.next_left_distance = rand()%2000 + 1;

			naviData.next_next_turn_type = -1;
			naviData.next_next_left_distance = 0;
			navigationPublisher.publish(naviData);
			ros::spinOnce();
		} else if (choosePub == -1) {
			break;
		}
		cnt++;
    }


    return 0;
}

