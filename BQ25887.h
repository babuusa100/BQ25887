#pragma once
#ifndef bq25887_h_
#define bq25887_h_
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// #define bitRead(value, bit) (((value) >> (bit)) & 0x01)
//#define bitSet(value, bit) ((value) |= (1UL << (bit)))
//#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define ENABLED         HIGH
#define DISABLED        LOW
namespace EmbeddedDevices
{
    
    template <int CELL>
    class BQ25887  
    {
        public :
            enum class TS_RANK
            {
                NORMAL = 0,
                WARM   = 2,
                COOL   = 3,
                COLD   = 5,
                HOT    = 6
            };
            
            enum class HIZ_MODE
            {
              DISABLE = 0,
              ENABLE = 1
            };
            
            enum class ILIM_FNC
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class VINDPM_RESET
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class BAT_DISCHG
            {
              DISABLE =0,
              ENABLE =1
            };

            enum class PFM_ODA_MODE
            {
              DISABLE =0,
              ENABLE =1
            };

            enum class FORCE_ICO_START
            {
              NOTFORCE =0,
              FORCE_START = 1
            };

            enum class FORCE_INDET_PSEL
            {
              NO_PSEL_DETECTION = 0,
              FORCE_PSEL_DETECTION = 1
            };

            enum class ICO_CTRL
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class TERMINATION_CTRL
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class STAT_PIN_FNC
            {
              ENABLE = 0,
              DISABLE = 1
            };

            enum class WATCHDG_TIMER
            {
              DISABLE = 0,
              SEC_40 = 1,
              SEC_80 = 2,
              SEC_160 = 3
            };

            enum class CHG_SAFETY_TIMER
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class FAST_CHG_TIMER
            {
              HRS_5 = 0,
              HRS_8 = 1,
              HRS_12 = 2,
              HRS_20 = 3
            };

            enum class SAFETY_TIMER_DPM_TREG
            {
              ALWAYS_COUNT = 0,
              TIMES_2X_SLOW = 1
            };

            enum class THERMAL_REG_TRSHOLD
            {
              C_60 = 0,
              C_80 = 1,
              C_100 = 2,
              C_120 = 3
            };

            enum class CHG_ENABLE_CONFIG
            {
              DISABLE = 0,
              ENABLE = 1
            };

            enum class PRE_FAST_CHG_TRSHOLD
            {
              TWO_POINT_EIGHT_V = 0,
              THREE_V =1
            };

            enum class PFM_MODE_CTRL
            {
              ENABLE = 0,
              DISABLE = 1
            };

            enum class I2C_WD_RST
            {
              NORMAL = 0,
              RST = 1
            };

            enum class TOP_OFF_TIMER
            {
              DISABLE = 0,
              MIN_FIFTEEN = 1,
              MIN_THIRTY = 2,
              MIN_FOURTY_FIVE = 3
            };

            enum class JEITA_VSET_HIGH_TEMP
            {
              CHG_SUSPEND = 0,
              VREG_EIGHT = 1,
              VREG_EIGHTPOINTTHREE = 2,
              VREG_CHANGED = 3
            };

            enum class JEITA_ISET_HIGH_TEMP
            {
              ICHG_PERC_FOURTY = 0,
              ICHG_PERC_HUNDRED = 1
            };

            enum class JEITA_ISET_LOW_TEMP
            {
              CHG_SUSPEND = 0,
              ICHG_PERC_TWENTY = 1,
              ICHG_PERC_FOURTY = 2,
              ICHG_PERC_HUNDRED = 3
            };

            enum class CHG_FAULT
            {
                NORMAL              = 0,
                INPUT_FAULT         = 8,
                THERMAL_SHUTDOWN    = 4,
                TIMER_EXPIRED       = 1
            };
            
            enum class VBUS_STAT
            {
                NO_INPUT        = 0,
                USB_HOST        = 1,
                USB_CDP         = 2,
                ADAPTER         = 3,
                POORSRC         = 4
            };

            enum class ICO_STAT
            {
              ICO_DISABLE      = 0,
              ICO_OPTIMISATION = 1,
              MAX_IP_CUR      = 2
            };
            
            enum class CHG_STAT
            {
                NOT_CHARGING    = 0,
                TRICKLE_CHARGE  = 1,
                PRE_CHARGE      = 2,
                FAST_CHARGE     = 3,
                TAPER_CHARGE    = 4,
                TOP_OFF_TIME_CHG= 5,
                CHARGE_DONE     = 6
            };

           
    private:
        float RtoTemp(float R)
        {
            float temperature = R / 10000.0f;
                  temperature = log(temperature);
                  temperature /= 3950.0f;
                  temperature += 1.f / 298.15f;
                  temperature = 1.f / temperature;
            return temperature - 273.25f;
        }
        const uint8_t I2C_ADDR = 0x6A;
        enum class REG
        {
            VCELL_REG           = 0x00,
            ICHG_ILIM           = 0x01,
            VINDPM              = 0x02,
            IINDPM              = 0x03,
            IPRECHG_TERM        = 0x04,
            CHG_CTRL1_TIME      = 0x05,
            CHG_CTRL2_VCELL     = 0x06,
            CHG_CTRL3_TIMER     = 0x07,
            CHG_CTRL4_JEITA     = 0x08,
            RESERVED            = 0x09,
            ICO_CUR_LIMIT       = 0x0A,
            CHG_STAT            = 0x0B,
            VBUS_ICO_STAT2      = 0x0C,
            NTC_STAT            = 0x0D,
            FAULT_STAT          = 0x0E,
            CHG_FLAG1           = 0x0F,
            CHG_FLAG2           = 0x10,
            FAULT_FLAG          = 0x11,
            CHG_MASK1           = 0x12,
            CHG_MASK2           = 0x13,
            FAULT_MASK          = 0x14,
            ADC_CTRL            = 0x15,
            ADC_FUNC_DISABLE    = 0x16,
            IBUS_ADC1           = 0x17,
            IBUS_ADC0           = 0x18,
            ICHG_ADC1           = 0x19,
            ICHG_ADC0           = 0x1A,
            VBUS_ADC1           = 0x1B,
            VBUS_ADC0           = 0x1C,
            VBAT_ADC1           = 0x1D,
            VBAT_ADC0           = 0x1E,
            VCELLTOP_ADC1       = 0x1F,
            VCELLTOP_ADC0       = 0x20,
            TS_ADC1             = 0x21,
            TS_ADC0             = 0x22,
            TDIE_ADC1           = 0x23,
            TDIE_ADC0           = 0x24,
            PART_INFO           = 0x25,
            VCELLBOT_ADC1       = 0x26,
            VCELLBOT_ADC0       = 0x27,
            CELL_BAL_CTRL1      = 0x28,
            CELL_BAL_CTRL2      = 0x29,
            CELL_BAL_STAT_CTRL  = 0x2A,
            CELL_BAL_FLAG       = 0x2B,
            CELL_BAL_MASK       = 0x2C
        };
        
       
        private:

            void write(const REG reg, const bool stop = true)
            {
                wire->beginTransmission(I2C_ADDR);
                wire->write((uint8_t)reg);
                wire->endTransmission(stop);
            }

            void write_(const REG reg, const uint8_t data, const bool stop = true)
            {
                wire->beginTransmission(I2C_ADDR);
                wire->write((uint8_t)reg);
                wire->write(data);
                wire->endTransmission(stop);
            }

            byte read(const REG reg)
            {
                byte data = 0;
                write(reg, false);
                wire->requestFrom((uint8_t)I2C_ADDR, (uint8_t)1);
                if(wire->available()==1)
                    data = wire->read();
                return data;
            }

            TwoWire* wire;
            float   VBUS, 
                    VCELL,
                    TSPCT, 
                    VBAT, 
                    ICHG,
                    IINDPM, 
                    VINDPM,
                    Temperature;

            VBUS_STAT   VBUS_STATUS;
            CHG_STAT    CHG_STATUS;
            TS_RANK     TS_RANK_;
            CHG_FAULT   CHG_FAULT_;
            ICO_STAT ICO_STATUS;
            
            ILIM_FNC ILIM_FNC_; 
            VINDPM_RESET VINDPM_RESET_; 
            BAT_DISCHG BAT_DISCHG_;
            PFM_ODA_MODE PFM_ODA_MODE_;
            FORCE_ICO_START FORCE_ICO_START_; 
            FORCE_INDET_PSEL FORCE_INDET_PSEL_; 
            ICO_CTRL ICO_CTRL_;
            TERMINATION_CTRL TERMINATION_CTRL_; 
            STAT_PIN_FNC STATUS_PIN_FNC_; 
            WATCHDG_TIMER WATCHDG_TIMER_; 
            THERMAL_REG_TRSHOLD THERMAL_REG_THRESHOLD_; 
            CHG_ENABLE_CONFIG CHG_ENABLE_CONFIG_; 
            JEITA_VSET_HIGH_TEMP JEITA_VSET_HIGH_TEMPERATURE_;
            
            
            bool VBUS_attached;
            bool VCELL_attached;
            bool thermal_regulation;
            void setADC_enabled(void)
            { 
                byte data = read(REG::ADC_CTRL);
                data |= (1UL << (2));       // start A/D convertion
                write_(REG::ADC_CTRL, data); 
            }
            void takeVBUSData(void)
            {
                byte data = read(REG::VBUS_ADC1);
                VBUS_attached = (((data) >> (7)) & 0x01) ? true : false;
                data &= ~ (1UL << (7));
                VBUS = 3.3f;
                VBUS  += (float) data *0.1f;
            }
            
            void takeVCELLData(void)
            {
                byte data = read(REG::VCELL_REG);
                VCELL_attached = (((data) >> (7)) & 0x01) ? true : false;
                data &= ~ (1UL << (7));
                VCELL  = 4.2f;
                VCELL += (float) data * 0.02f;
            }

            void takeVBATData()//keep for addition but remove finally if not in use
            {
                byte data = read(REG::BATV);
                thermal_regulation = (((data) >> (7)) & 0x01) ? true : false;
                data &= ~ (1UL << (7));
                VBAT  =  (float)data * 0.02f;
                VBAT += 2.304f;
            }

            void takeTSPCTData(void)//keep for addition but remove finally if not in use
            {
                int data = (int) read(REG::TSPCT);
                TSPCT = (float) data*0.465f + 21.f;
                
            }
            void takeICHGData(void)
            {
                uint8_t data = read(REG::ICHG_ILIM);
                ICHG = (float) data * 0.05f;
            }

            void takeVBUSSTAT(void)
            {
                uint8_t data = read(REG::VBUS_ICO_STAT2);
            // parsing VBUS_STAT
                if(((data) >> (4)) & 0x01)
                    VBUS_STATUS = VBUS_STAT::POORSRC;
                else
                {
                    if(((data) >> (3)) & 0x01)
                        VBUS_STATUS = VBUS_STAT::ADAPTER;
                    else if( (((data) >> (2)) & 0x01) )
                        VBUS_STATUS = VBUS_STAT::USB_CDP;
                    else if( (((data) >> (1)) & 0x01) )
                        VBUS_STATUS = VBUS_STAT::USB_HOST;
                    else
                        VBUS_STATUS = VBUS_STAT::NO_INPUT;
                };
            }
            // parsing CHG_STAT
            
            void takeCHGSTAT(void) //FUTURE SET CODE BELOW
            {
              uint8_t data =read(REG::CHG_STAT1);
                if(((data) >> (6)) & 0x01)
                {
                    if(((data) >> (5)) & 0x01)  {CHG_STATUS = CHG_STAT::TOP_OFF_TIME_CHG;}
                    else                        {CHG_STATUS = CHG_STAT::CHARGE_DONE;};
                }
                else
                {
                    if(((data) >> (4)) & 0x01) 
                        CHG_STATUS = CHG_STAT::TAPER_CHARGE;
                    else if( (((data) >> (3)) & 0x01) )                      
                        CHG_STATUS = CHG_STAT::FAST_CHARGE;
                    else if( (((data) >> (2)) & 0x01) )                      
                        CHG_STATUS = CHG_STAT::PRE_CHARGE;
                    else if( (((data) >> (1)) & 0x01) )                      
                        CHG_STATUS = CHG_STAT::TRICKLE_CHARGE;
                    else                      
                        CHG_STATUS = CHG_STAT::NOT_CHARGING;
   
                };
            // parsing VCELL_status
               
            }

            void takeTempData(void)
            {
                uint8_t data = read(REG::FAULT_);
            // parsing temperature rank
                if(((data) >> (2)) & 0x01)      // hot or cold
                {
                    if(((data) >> (1)) & 0x01)  // hot
                        {TS_RANK_ = TS_RANK::HOT;}
                    else                        // cold
                        {TS_RANK_ = TS_RANK::COLD;};
                }
                else                            // cool, warm or normal
                {
                    if(((data) >> (1)) & 0x01)  // warm or cool
                    {
                        if(((data) >> (0)) & 0x01)  // cool
                            {TS_RANK_ = TS_RANK::COOL;}
                        else                        // warm
                            {TS_RANK_ = TS_RANK::WARM;};
                    }
                    else                        // normal
                        {TS_RANK_ = TS_RANK::NORMAL;};
                }
            }
            // parsing charging  fault
           
            void takeFaultData (void)
            {
                uint8_t data = read(REG::CHG_FAULT);
                if(((data) >> (8)) & 0x01)  // termal shutdown or timer expired
                {
                    if(((data) >> (4)) & 0x01) 
                        {CHG_FAULT_ = CHG_FAULT::THERMAL_SHUTDOWN;}
                    else
                        {CHG_FAULT_ = CHG_FAULT::INPUT_FAULT;};
                }
                else                        // normal or input fault
                {
                    if(((data) >> (1)) & 0x01)
                        CHG_FAULT_ = CHG_FAULT::TIMER_EXPIRED;
                    else
                        CHG_FAULT_ = CHG_FAULT::NORMAL;
                }
            }

            //DMA_HandleTypeDef s_DMAHandle;

        public:
            BQ25887(TwoWire& w) : wire(&w){};
            void begin(void) {setADC_enabled();}
            void properties(void)
            {
            //  read register status
                takeVBUSSTAT();
                takeTempData();
            // read register ADC
                takeVBUSData();
                takeVCELLData();
                takeVBATData();
                takeTSPCTData();
                takeICHGData();
                setADC_enabled();
                takeFaultData();
                takeICHGSTAT();
            }

            float getVBUS(void) {return VBUS;}
            float getVCELL(void) {return VCELL;}
            float getVBAT(void) {return VBAT;}
            float getICHG(void) {return ICHG;}
            float getTSPCT(void)
            
            {
                int data = (int) read(REG::TSPCT);
                float tmp = (float) data*0.465f + 21.f;
                this->TSPCT = tmp;
                return tmp;
            }

            float getTemperature(void)
            {
                float VTS = 5.0f * TSPCT / 100.f;
                float RP  = ( VTS * 5230.f ) / (5.f - VTS);
                float NTC = ( RP * 30100.f ) / ( 30100.f - RP);
                return RtoTemp(NTC);
            }

            CHG_STAT getCHG_STATUS(void){return CHG_STATUS;}
            VBUS_STAT getVBUS_STATUS(void){return VBUS_STATUS;}
            
            TS_RANK getTemp_Rank(void){return TS_RANK_;}
            CHG_FAULT getCHG_Fault_STATUS(void){return CHG_FAULT_;}
            
            float getFast_Charge_Current_Limit(void)
            {
                byte data = read(REG::ICHG_ILIM);
                data &= ~(1UL << 7);
                return (float) data * 0.05f;
            }

            float getInput_Current_Limit(void)
            {
                byte data = read(REG::IINDPM);
                data &= ~(1UL << 6);
                data &= ~(1UL << 5);
                return (float) data * 0.05f;
            }

            float getPreCharge_Current_Limit(void)
            {
                byte data = read(REG::IPRECHG_TERM);
                data &= 0b11110000;
                return (float)(data>>4)*0.05f+0.05f;
            }
            
            float getTermination_Current_Limit(void)
            {
                byte data = read(REG::IPRECHG_TERM);
                data &= 0b00001111;
                return (float)(data)*0.05f+0.05f;
            }

            float getCharge_Voltage_Limit(void)
            {
                byte data = read(REG::VCELL_REG);
                data  = data >> 2;
                return (float)(data)*0.005f+3.40f;
            }

            void setMinVBUS(float volt = 4.3f)
            {
                byte data = read(REG::VINDPM);
                data &= 0b10000000;
                if( volt > 3.9f )
                {
                    volt -=3.9;
                    volt = volt * 10;
                    byte temp_ = (byte)volt;
                    data |= temp_;
                }
                write_(REG::VINDPM,data);
            }

            void setFast_Charge_Current_Limit(float cur)
            {
                cur = (cur > 3.008f)? 3.008f:((cur < 0 )? 0 : cur);
                cur /= 8.128;
                cur *= 127;
                byte reg = read(REG::ICHG_ILIM);
                byte tmp = 128 | (byte) cur;
                write_(REG::ICHG_ILIM,tmp);
            }
            
            void setInput_Current_Limit(float cur)
            {
                cur  = (cur > 3.3f)? 3.3f:((cur < 100 )? 100 : cur);
                cur -= 100;
                cur /= 3.15;
                cur *= 63;
                byte data = read(REG::IINDPM);
                byte tmp = (((data) >> (6)) & 0x01) | (((data) >> (5)) & 0x01)  | (byte) cur;
                write_(REG::IINDPM,tmp);
            }

            void setPreCharge_Current_Limit(float cur)
            {
                cur  = (cur > 0.8f) ? 0.8f:((cur <0.05f )? 0.05f : cur);
                cur -= 0.05f;
                cur /= 0.96f;
                cur *= 0x0f;
                byte data = read(REG::IPRECHG_TERM);
                data &=0b00001111;
                data |= (byte)cur<<4;
                write_(REG::IPRECHG_TERM,data);
            }
            void setTermination_Current_Limit(float cur)
            {
                cur  = (cur > 0.8f) ? 0.8f:((cur <0.05f )? 0.05f : cur);
                cur -= 0.05f;
                cur /= 0.96f;
                cur *= 0x0f;
                byte data = read(REG::IPRECHG_TERM);
                data &=0b11110000;
                data |= (byte)cur;
                write_(REG::IPRECHG_TERM,data);
            }

            void setCharge_Voltage_Limit(float cur)
            {
                cur  = (cur > 4.608f) ? 4.608f:((cur <3.40f )? 3.40f : cur);
                cur -= 3.40f;
                cur /= 1.008f;
                cur *= 63;
                byte data = read(REG::VCELL_REG);
                data &=0b00000011;
                data |= (byte)cur<<2;
                write_(REG::VCELL_REG,data);
            }
            void setBatLoad(uint8_t mode)
            {
                byte data = read(REG::VINDPM);
                if(mode == DISABLED)
                {
                    data &= ~(1UL << 6UL);
                }
                else
                {
                    data |= (1UL << 6UL);
                }
                data |= (1UL << 1UL);
                data |= (1UL << 2UL);
                data |= (1UL << 3UL);
                
                write_(REG::VINDPM,data);
            }
            
            void setChargeEnable(uint8_t mode)
            {
                byte data = read(REG::CHG_CTRL2_VCELL);
                if(mode == DISABLED)
                {
                    data &= ~(1U << 3U);
                }
                else
                {
                    data |= (1U << 3U);
                }
                write_(REG::CHG_CTRL2_VCELL,data);
            }
            void setForceICO(uint8_t mode)
            {
                byte data = read(REG::IINDPM);
                if(mode == DISABLED)
                {
                    data &= ~(1U << 7U);
                    data &= ~(1U << 0U);
                    data &= ~(1U << 1U);
                }
                else
                {
                    data |= (1U << 7U);
                    data |= (1U << 0U);
                    data |= (1U << 1U);
                }
                write_(REG::IINDPM,data);
            }
    };
}
using BQ25887 = EmbeddedDevices::BQ25887<1>;
#endif