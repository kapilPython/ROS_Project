#include "yaml-cpp/yaml.h"
#include "yaml-cpp/exceptions.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#define DEBUG 0 // change this to 1 if you want debug messages on console
class GoalDatabase{
    private:
        std::map<int,std::vector<float> > m_goalDatabase;
    public:
        // constructor
        GoalDatabase(){
        }
        std::map<int,std::vector<float> > getDatabase(){
            return m_goalDatabase;
        }
        /**
            in - std::string 
            out - bool
            this function picks up data from yaml and saves it into local map
        */
        bool createDatabase(std::string path){
            bool NoParseError = true;
            bool NoDataError = true;
            try{
                YAML::Node goalData = YAML::LoadFile(path);
                std::vector<float> tempVec;
                if(DEBUG == 1){
                    std::cout<< goalData.IsSequence()<<std::endl;
                    std::cout << goalData << std::endl;
                }
                for(unsigned int i=0; i < goalData.size(); i++)
                {
                    if(DEBUG == 1){
                        std::cout << (goalData[i])["x"] << std::endl;  
                        std::cout << (goalData[i])["y"] << std::endl;
                        std::cout << (goalData[i])["w"] << std::endl;
                    }
                    m_goalDatabase.insert(std::pair<int,std::vector<float> >(i,tempVec));
                    float x,y,w;
                    try{
                        x = ((goalData[i])["x"]).as<float>();
                        y = ((goalData[i])["y"]).as<float>();
                        w = ((goalData[i])["w"]).as<float>();
                    }
                    catch(YAML::TypedBadConversion<float>& e){
                        std::cout<<"OOPS! There is a problem with the YAML"<<std::endl;
                        std::cout<<e.what()<<std::endl;
                        NoDataError = false;     
                    }
                    m_goalDatabase[i].push_back(x);
                    m_goalDatabase[i].push_back(y);
                    m_goalDatabase[i].push_back(w);
                }
            }
            catch(YAML::ParserException& e){
                std::cout<<"OOPS! There is a problem with the YAML"<<std::endl;
                std::cout<<e.what()<<std::endl;
                NoParseError = false;
            }
            if(NoDataError && NoParseError){
                return true;
            }
            else{
                return false;
            }
        }
        // This function prints the local map on console
        void print(){
            //std::map<int,vector<float>>
            for(std::map<int,std::vector<float> >::iterator it = m_goalDatabase.begin();it != m_goalDatabase.end(); ++it){
                std::cout<<it->first<<"\n";
                std::cout<<(it->second)[0]<<" ";
                std::cout<<(it->second)[1]<<" ";
                std::cout<<(it->second)[2]<<"\n";
            }
        }
};