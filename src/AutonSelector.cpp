#include "AutonSelector.h"
#include "Robot.h"

extern lv_font_t lv_font_dejavu_40;

lv_style_t bTGL_Pr;
lv_style_t bTGL_PrRED;
lv_style_t bTGL_PrBLUE;
lv_style_t bTGL_PrGREEN;


inline lv_coord_t percentX(double percent) {
	return 480.0 / 100 * percent;
}

inline lv_coord_t percentY(double percent) {
	return 240.0 / 100 * percent;
}

static uint32_t combine(uint16_t low, uint16_t high) {
	return (static_cast<uint32_t>(high) << 16) + static_cast<uint32_t>(low);
}

static std::pair<uint16_t, uint16_t> separate(uint32_t x) {
	return std::make_pair(static_cast<uint16_t>(x >> 16), static_cast<uint16_t>(x));
}

AutonSelector::Button::Button(lv_obj_t* parent, const char* name, Auton auton, Alliance alliance) {
	btn = lv_btn_create(parent);
	label = lv_label_create(btn);
	lv_label_set_text(label, name);
	lv_obj_center(label);
	lv_obj_add_event_cb(
	        btn,
	        [](lv_event_t* event) {
		        // higher bits = alliance
		        // lower bits = auton
		        auto decodedData = separate((uint32_t) lv_event_get_user_data(event));
		        robotInstance->curAlliance = static_cast<Alliance>(decodedData.first);
		        robotInstance->curAuton = static_cast<Auton>(decodedData.second);
	        },
	        LV_EVENT_ALL, (void*) combine(static_cast<uint16_t>(auton), static_cast<uint16_t>(alliance)));
}

void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
	lv_obj_set_pos(btn, x, y);
}

void AutonSelector::Button::setSize(lv_coord_t w, lv_coord_t h) {
	lv_obj_set_size(btn, w, h);
}

AutonSelector::AutonSelector() {
	pros::lcd::initialize();
	lcdScreen = lv_scr_act();
}

void AutonSelector::run() {
	selectorScreen = lv_obj_create(NULL);


	// create buttons

	// blue -> ringside elim
	// B -> ringside qual
	// R -> ringside elim
	// R -> ringisde qual

	blueRingsideElim = Button(selectorScreen, "Blue Elim", Auton::ELIM, Alliance::BLUE);
	blueRingsideQual = Button(selectorScreen, "Blue Qual", Auton::QUAL, Alliance::BLUE);
	redRingsideElim = Button(selectorScreen, "Blue Elim", Auton::ELIM, Alliance::RED);
	redRingsideQual = Button(selectorScreen, "Blue Qual", Auton::QUAL, Alliance::RED);

	lv_scr_load(selectorScreen);

	// unstalls the entire pipeline incase someone forgets to select an auton
	while (robotInstance->curAuton == Auton::NONE && pros::competition::is_disabled()) { pros::delay(10); }
}