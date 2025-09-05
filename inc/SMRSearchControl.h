#pragma once

class SMRSearchControl {
public:
	SMRSearchControl();
	~SMRSearchControl();

	//whenever you start or restart tracking an SMR
	void startSearch();
	void startSearch(float distanceGuess);

	//update search distance, search algorithm, other search actions
	void updateSearch(bool locked, bool targetFound, bool smrHitByLaser, float laserImgAz, float laserImgEl, float smrImgAz, float smrImgEl, float smrDistance);

	//report if image processor needs to search for red reflection (used if we've lost track of target SMR when laser approached it)
	//bool useRedSearch() { return shouldUseRed; };

	void laserObserved();
	void smrObserved();

	float getLaserObservedDistance() { return observedLaserDistance; }
	float setLaserObservedDistance(float distance) { observedLaserDistance = distance; }

	float DistEstimatePow(int x, int nmode );
	const float MAXIMUM_EST_DISTANCE = 80.0;
	const float m_NearCalibDistance = 1.5;

	//report if search movement should be ignored in favor of a manual jog by movement (if we've lost track of target SMR for too long)
	bool jogRequired();
	bool JogToStartSpiral();
	int InitialJogCount;
	//if not overridden, get the current search distance for moving to the target SMR
	float getCurrentSearchDistance();

	float getCurrentSearchRadius();

	void reportLock();
	bool lockedOnSMR;
	int redBeamCount;
private:

	bool wantTarget;
	int searchCount;
	int searchCountELPlus;
	int searchCountELMinus;
	float searchDistance;
	//maximum value for the search parameter, corresponds to a location on the target line near the edge of the image)
	float minSearchDistance;
	float maxSearchDistance;
	bool isFirstIteration;

	//int targetMissingCount;
	//bool shouldUseRed;
	//float lastSMRAz, lastSMREl;
	
	bool allowJog;
	bool wasHitByLaser;
	float searchRadius;
	bool dist_inc = false;
	bool dist_dec = false;
	float observedLaserDistance;

	void updateSearchDistance(float estimatedDistance, bool smrClose);
};
