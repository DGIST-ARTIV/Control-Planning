/*******************************************************
 * Copyright (C) 2020-2021 김민종(Minjong Kim) <tlemsl@dgist.ac.kr or tlemsl8963@gmail.com>
 * Version 1.0.0
 * This file is part of DGIST ARTIV control planneig team(https://github.com/DGIST-ARTIV/Control-Planning).
 *
 * DGIST ARTIV can not be copied and/or distributed without the express
 * permission of 김민종(Minjong Kim)
 * For a temporary license for autonomous vehicle
 *******************************************************/
#include "scc/logger.h"

void logger::INFO(std::string data){
	if(!flag)	return;
	else	std::cout << GREEN << data << RESET << std::endl;
}
void logger::WARNING(std::string data){
	if(!flag)	return;
	else	std::cout << YELLOW << data << RESET << std::endl;
}
void logger::ERROR(std::string data){
	if(!flag)	return;
	else	std::cout<< RED << data <<std::endl;
}
void logger::FATAL(std::string data){
	if(!flag)	return;
	else 	std::cout<<BOLDRED<<data<<std::endl;
}