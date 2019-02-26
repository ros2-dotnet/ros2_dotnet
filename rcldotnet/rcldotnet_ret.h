// Copyright 2016-2018 Esteve Fernandez <esteve@apache.org>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLDOTNET_RET_H
#define RCLDOTNET_RET_H

const int RCLDOTNET_OK = 0;
const int RCLDOTNET_ERROR = 1;
const int RCLDOTNET_TIMEOUT = 2;
const int RCLDOTNET_BADALLOC = 10;
const int RCLDOTNET_INVALIDARGUMENT = 11;
const int RCLDOTNET_ALREADYINIT = 100;
const int RCLDOTNET_NOTINIT = 101;
const int RCLDOTNET_MISMATCHEDRMWID = 102;
const int RCLDOTNET_TOPICNAMEINVALID = 103;
const int RCLDOTNET_SERVICENAMEINVALID = 104;
const int RCLDOTNET_UNKNOWNSUBSTITUTION = 105;
const int RCLDOTNET_NODEINVALID = 200;
const int RCLDOTNET_NODEINVALIDNAME = 201;
const int RCLDOTNET_NODEINVALIDNAMESPACE = 202;
const int RCLDOTNET_PUBLISHERINVALID = 300;
const int RCLDOTNET_SUBSCRIPTIONINVALID = 400;
const int RCLDOTNET_SUBSCRIPTIONTAKEFAILED = 401;
const int RCLDOTNET_CLIENTINVALID = 500;
const int RCLDOTNET_CLIENTTAKEFAILED = 501;
const int RCLDOTNET_SERVICEINVALID = 600;
const int RCLDOTNET_SERVICETAKEFAILED = 601;
const int RCLDOTNET_TIMERINVALID = 800;
const int RCLDOTNET_TIMERCANCELED = 801;
const int RCLDOTNET_WAITSETINVALID = 900;
const int RCLDOTNET_WAITSETEMPTY = 901;
const int RCLDOTNET_WAITSETFULL = 902;

#endif // RCLDOTNET_RET_H
