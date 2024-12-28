#include "AutonSelector.h"
#include "Robot.h"

extern lv_font_t lv_font_dejavu_40;

static lv_style_t btnRedStyle;


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

static void btnCallback(lv_event_t* event) {
	// higher bits = alliance
	// lower bits = auton
	auto decodedData = separate((uint32_t) lv_event_get_user_data(event));
	robotInstance->curAlliance = static_cast<Alliance>(decodedData.first);
	robotInstance->curAuton = static_cast<Auton>(decodedData.second);
}

AutonSelector::Button::Button(lv_obj_t* parent, const char* name, Auton auton, Alliance alliance) {
	btn = lv_btn_create(parent);
	label = lv_label_create(btn);
	lv_label_set_text(label, name);
	lv_obj_center(label);
	lv_obj_add_event_cb(btn, btnCallback, LV_EVENT_PRESSED,
	                    (void*) combine(static_cast<uint16_t>(auton), static_cast<uint16_t>(alliance)));
}

void AutonSelector::Button::setPos(lv_coord_t x, lv_coord_t y) {
	lv_obj_set_pos(btn, x, y);
}

void AutonSelector::Button::setSize(lv_coord_t w, lv_coord_t h) {
	lv_obj_set_size(btn, w, h);
}

void AutonSelector::Button::setStyle(lv_style_t* style) {
	lv_obj_add_style(btn, style, 0);
}

AutonSelector::AutonSelector() {
	pros::lcd::initialize();
	lcdScreen = lv_scr_act();
}

void AutonSelector::run() {
	selectorScreen = lv_obj_create(NULL);

	lv_style_init(&btnRedStyle);
	lv_style_set_bg_color(&btnRedStyle, lv_color_hex(0xf21e0f));
	lv_style_set_bg_opa(&btnRedStyle, LV_OPA_COVER);


	// create buttons

	// blue -> ringside elim
	// B -> ringside qual
	// R -> ringside elim
	// R -> ringisde qual

	blueRingsideElim = Button(selectorScreen, "Blue Elim", Auton::ELIM, Alliance::BLUE);
	blueRingsideElim.setPos(percentX(10), percentY(5));
	blueRingsideElim.setSize(150, 100);
	blueRingsideQual = Button(selectorScreen, "Blue Qual", Auton::QUAL, Alliance::BLUE);
	blueRingsideQual.setPos(percentX(65), percentY(5));
	blueRingsideQual.setSize(150, 100);
	redRingsideElim = Button(selectorScreen, "Red Elim", Auton::ELIM, Alliance::RED);
	redRingsideElim.setPos(percentX(10), percentY(50));
	redRingsideElim.setSize(150, 100);
	redRingsideElim.setStyle(&btnRedStyle);
	redRingsideQual = Button(selectorScreen, "Red Qual", Auton::QUAL, Alliance::RED);
	redRingsideQual.setPos(percentX(65), percentY(50));
	redRingsideQual.setSize(150, 100);
	redRingsideQual.setStyle(&btnRedStyle);

	lv_scr_load(selectorScreen);

	// unstalls the entire pipeline incase someone forgets to select an auton
	while (robotInstance->curAuton == Auton::NONE && pros::competition::is_disabled()) { pros::delay(10); }
	lv_scr_load(lcdScreen);
}