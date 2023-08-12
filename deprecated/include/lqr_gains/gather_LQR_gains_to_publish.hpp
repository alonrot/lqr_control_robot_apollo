#ifndef GATHER_LQR_GAINS_TO_PUBLISH_H
#define GATHER_LQR_GAINS_TO_PUBLISH_H

#include "lqr_gains/read_LQR_gains_from_file.hpp"

#define RATE 50

class GatherLQRGainsToPublish{

public:

	GatherLQRGainsToPublish(ReadLQRGains * safe, ReadLQRGains * search, std::string NODE_NAME);
	~GatherLQRGainsToPublish();
  bool run(void);

private:
	ReadLQRGains * 	safe;
	ReadLQRGains * 	search;
	std::string 	NODE_NAME;
};

#endif /* GATHER_LQR_GAINS_TO_PUBLISH_H */