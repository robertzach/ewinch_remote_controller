/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/


//send by transmitter
struct LoraTxMessage {
   uint8_t id;
   uint8_t pullValue;
   uint8_t pullValueBackup;
};
//send by receiver (acknowledgement)
struct LoraRxMessage {
   uint8_t pullValue;
   uint8_t tachometer;    //*10 --> in m
   uint8_t dutyCycleNow;
   uint8_t vescBatteryPercentage;
   uint8_t vescTempMotor;
};
