#ifndef DisplayFacade_H
#define DisplayFacade_H

class DisplayFacade {
public:
    // --- Singleton access ---
    static DisplayFacade& instance()
    {
        static DisplayFacade facade;
        return facade;
    }

    // --- Getters/Setters ---
    void setSatellites(int val) { satellites = val; }
    int getSatellites() const { return satellites; }

    void setBtConnected(bool val) { btConnected = val; }
    bool getBtConnected() const { return btConnected; }

    void setSpeedKmph(float val) { speedKmph = val; }
    float getSpeedKmph() const { return speedKmph; }

    void setPb(unsigned long val) { pb = val; }
    unsigned long getPb() const { return pb; }

    void setLapTime(unsigned long val) { lapTime = val; }
    unsigned long getLapTime() const { return lapTime; }

    void setBestLapTime(unsigned long val) { bestLapTime = val; }
    unsigned long getBestLapTime() const { return bestLapTime; }

    void setStoredBestLap(unsigned long val) { storedBestLap = val; }
    unsigned long getStoredBestLap() const { return storedBestLap; }

private:
    // --- Private constructor (no copy, no move) ---
    DisplayFacade() = default;
    DisplayFacade(const DisplayFacade&) = delete;
    DisplayFacade& operator=(const DisplayFacade&) = delete;

    int satellites = 0;
    bool btConnected = false;
    float speedKmph = 0.0f;
    unsigned long pb = 0;
    unsigned long lapTime = 0;
    unsigned long bestLapTime = 0;
    unsigned long storedBestLap = 0;
};
#endif
