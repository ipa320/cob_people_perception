/*
 * global_config.h
 *
 *  Created on: May 12, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_LEG_DETECTOR_SRC_GLOBAL_CONFIG_H_
#define PEOPLE_LEG_DETECTOR_SRC_GLOBAL_CONFIG_H_

class GlobalConfig
{
   public:
     static GlobalConfig* exemplar();

   protected:
     GlobalConfig() {}

   private:
     static GlobalConfig *instanz;
     GlobalConfig( const GlobalConfig& );
};

GlobalConfig* GlobalConfig::instanz = 0;

GlobalConfig* GlobalConfig::exemplar()
{
  if( instanz == 0 )
    instanz = new GlobalConfig();
  return instanz;
}

#endif /* PEOPLE_LEG_DETECTOR_SRC_GLOBAL_CONFIG_H_ */
