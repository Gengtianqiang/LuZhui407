/* * Copyright (c) 2021 BlackWalnut Labs., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef _AK09916_ENUMERATIONS_H_
#define _AK09916_ENUMERATIONS_H_

typedef enum
{
	AK09916_mode_power_down = 0x00,
	AK09916_mode_single = (0x01 << 0),
	AK09916_mode_cont_10hz = (0x01 << 1),
	AK09916_mode_cont_20hz = (0x02 << 1),
	AK09916_mode_cont_50hz = (0x03 << 1),
	AK09916_mode_cont_100hz = (0x04 << 1),
	AK09916_mode_self_test = (0x01 << 4),
} AK09916_mode_e;

#endif // _AK09916_ENUMERATIONS_H_
