/*#pragma once
#include "Robot.h"
#include <atomic>
#include <stdint.h>
#include <vector>

extern "C" {
typedef struct _lv_obj_t lv_obj_t;
typedef int16_t lv_coord_t;
typedef uint8_t lv_res_t;
typedef lv_res_t (*lv_action_t)(struct _lv_obj_t* obj);
}

enum class Auton : uint32_t;

#define sAutonSelector AutonSelector::getInstance()

// half assed auton selector
class AutonSelector {
private:
    class Button {
    private:
        lv_obj_t* btn = nullptr;
        lv_obj_t* label = nullptr;

    public:
        Button() = default;
        Button(lv_obj_t* parent, const char* name, lv_action_t btnCallback = nullptr, uint32_t optData = 0);
        void setPos(lv_coord_t x, lv_coord_t y);
        void setSize(lv_coord_t w, lv_coord_t h);
        void setStyle(void* style);
        void setHidden(bool en);
    };

    lv_obj_t* lcdScreen = nullptr;// pros::lcd lvgl screen
    lv_obj_t* selectorScreen = nullptr;

    // tabs
    lv_obj_t* tabview = nullptr;
    lv_obj_t* matchTypeTab = nullptr;// tab to decide whether or not to select for qual/elim
    lv_obj_t* sideTab = nullptr;     // to decide between far/right autons
    lv_obj_t* autonsTab = nullptr;   // tab to select corresponding autons given the previously selected info

    // buttuons for matchTypeTab
    Button elimBtn;
    Button qualsBtn;

    // Buttons for sideTab
    Button farBtn;
    Button closeBtn;

    // Slider for auton selection
    lv_obj_t* autonRoller = nullptr;

    // auton selection buttons
    lv_obj_t* farElimAutonsParent = nullptr;
    lv_obj_t* closeElimAutonsParent = nullptr;

    lv_obj_t* farQualAutonsParent = nullptr;
    lv_obj_t* closeQualAutonsParent = nullptr;

    Button cycleLeftButton; // button to cycle left for shown auton
    Button cycleRightButton;// button to cycle right for shown auton
    int autonIndex = 0;     // which auton to show within the list

    std::vector<Button> farElimAutons;
    std::vector<Button> closeElimAutons;

    std::vector<Button> farQualAutons;
    std::vector<Button> closeQualAutons;

    std::atomic_bool choseMatchType = false;
    std::atomic_bool choseSide = false;
    std::atomic_bool isFar = false;

    AutonSelector() = default;
    AutonSelector(const AutonSelector&) = delete;
    AutonSelector& operator=(const AutonSelector&) = delete;

    inline std::vector<Button>& getCurrentAutons() {
        return sRobot.isQual ? isFar ? farQualAutons : closeQualAutons : isFar ? farElimAutons : closeElimAutons;
    }

    inline lv_obj_t* getCurrentAutonsParent() {
        return sRobot.isQual ? isFar ? farQualAutonsParent : closeQualAutonsParent
               : isFar       ? farElimAutonsParent
                             : closeElimAutonsParent;
    }

public:
    inline static AutonSelector& getInstance() {
        static AutonSelector INSTANCE;

        return INSTANCE;
    }

    void initialize();

    // Doesn't add auton to auton selector if it hasn't been initialized!!!
    void addAuton(const char* name, bool isQual, bool isFar, Auton auton);

    // stalls the entire function until an auton is selected
    void showSelector();
    void restoreLCD();
};*/