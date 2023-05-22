#include "plugin.hpp"
#include "dsp/digital.hpp"
#define TRIG_TIME 1e-3f
#define DEBUG_PRINT 1

struct E325BCE : Module {
	enum ParamId {
		STEPS_PARAM,
		NUMSTEPS_PARAM,
		PULSES_PARAM,
		MODE_PARAM,
		OFFSET_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		CLOCK_INPUT,
		RESET_INPUT,
		STEPSCV_INPUT,
		PULSESCV_INPUT,
		OFFSETCV_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		AUXOUT_OUTPUT,
		MAINOUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		CLOCKLED_LIGHT,
		RESETLED_LIGHT,
		AUXOUTLED_LIGHT,
		MAINOUTLED_LIGHT,
		LIGHTS_LEN
	};
	dsp::PulseGenerator pgen;
	dsp::PulseGenerator auxPgen;
	dsp::SchmittTrigger clockDetector;
	dsp::SchmittTrigger resetDetector;
	int stepNr = 0;
	int rhythm[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
	int rhythm2[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
    int rhythm3[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};	
	int lastNumSteps = 0;
	int numSteps = 16;

	E325BCE() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(STEPS_PARAM, 1.f, 16.f, 16.f, "Steps");
		paramQuantities[STEPS_PARAM]->snapEnabled = true;
		configParam(NUMSTEPS_PARAM, 0.f, 1.f, 0.f, "16/32");
		configParam(PULSES_PARAM, 0.f, 16.f, 16.f, "Pulses");
		paramQuantities[PULSES_PARAM]->snapEnabled = true;
		configParam(MODE_PARAM, 0.f, 1.f, 0.f, "Norm/XOR!!!");
		configParam(OFFSET_PARAM, 0.f, 16.f, 0.f, "Offset");
		paramQuantities[OFFSET_PARAM]->snapEnabled = true;
		configInput(CLOCK_INPUT, "Clock In");
		configInput(RESET_INPUT, "Reset In");
		configInput(STEPSCV_INPUT, "Steps CV");
		configInput(PULSESCV_INPUT, "Pulses CV");
		configInput(OFFSETCV_INPUT, "Offset CV");
		configOutput(AUXOUT_OUTPUT, "Aux Pulse");
		configOutput(MAINOUT_OUTPUT, "Pulse");
	}

	void process(const ProcessArgs& args) override;
	void euclid(int steps, int pulses){
		for (int y = 0; y<steps; y++) rhythm[y] = 0;
		int bucket = 0;
		for (int a=0; a < steps; a++){
		    bucket += pulses;
		    if (bucket >= steps) {
		      bucket -= steps;
		      rhythm[a] = 1;
		    } else rhythm[a] = 0;
		}
		int tempArray[32] = {0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0};
		tempArray[0] = rhythm[numSteps-1];
		for(int x = 1; x<numSteps; x++) tempArray[x] = rhythm[x-1];
		for(int y = 0; y<numSteps; y++) rhythm[y] = tempArray[y];
	}
	void euclid2(int steps, int pulses){

		for (int y = 0; y<steps; y++) rhythm2[y] = 0;
		int bucket = 0;
		for (int a=0; a < steps; a++){
		    bucket += pulses;
		    if (bucket >= steps) {
		      bucket -= steps;
		      rhythm2[a] = 1;
		    } else rhythm2[a] = 0;
		}
		int tempArray[32] = {0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0};
		tempArray[0] = rhythm2[numSteps-1];
		for(int x = 1; x<numSteps; x++) tempArray[x] = rhythm2[x-1];
		for(int y = 0; y <numSteps; y++) rhythm2[y] = tempArray[y];
	}			
			
};

void E325BCE::process(const ProcessArgs &args) {
	if ((int)params[NUMSTEPS_PARAM].getValue() != lastNumSteps) {
		lastNumSteps = (int)params[NUMSTEPS_PARAM].getValue();
		if ((int)params[NUMSTEPS_PARAM].getValue() == 0) {
			configParam(STEPS_PARAM, 1.f, 16.f, 16.f, "Steps");
			configParam(PULSES_PARAM, 0.f, 16.f, 16.f, "Pulses");
			configParam(OFFSET_PARAM, 0.f, 16.f, 0.f, "Offset");
			numSteps = 16;
		}
		else {
			configParam(STEPS_PARAM, 1.f, 32.f, 32.f, "Steps");
			configParam(PULSES_PARAM, 0.f, 32.f, 32.f, "Pulses");
			configParam(OFFSET_PARAM, 0.f, 32.f, 0.f, "Offset");
			numSteps = 32;
		}
		paramQuantities[STEPS_PARAM]->snapEnabled = true;
		paramQuantities[PULSES_PARAM]->snapEnabled = true;
		paramQuantities[OFFSET_PARAM]->snapEnabled = true;	
	}
	euclid(numSteps,(int)(params[STEPS_PARAM].getValue())+inputs[STEPSCV_INPUT].getVoltage());
	euclid2(numSteps,(int)(params[PULSES_PARAM].getValue())+inputs[PULSESCV_INPUT].getVoltage());
	for (int w = 0; w<numSteps; w++){
		rhythm3[w] = (rhythm[w] ^ rhythm2[w]);
	}
	if (resetDetector.process(inputs[RESET_INPUT].getVoltage())) stepNr = 0;

	if (clockDetector.process(inputs[CLOCK_INPUT].getVoltage())) {

			int offsetVal = stepNr + params[OFFSET_PARAM].getValue();
			if (offsetVal > numSteps-1) {
				offsetVal = offsetVal - numSteps-1;
			}
			if (rhythm3[offsetVal])
				{
				pgen.trigger(TRIG_TIME);
				}
			else {
				auxPgen.trigger(TRIG_TIME);
			}
			stepNr = (stepNr + 1);

			if (stepNr > numSteps-1) stepNr = 0;
			printf("=-=-=-=-=-=-=-=- =-=-\n");
			if (DEBUG_PRINT) {
			//printf("Step: %d\n", stepNr);
			for (int e = 0; e < numSteps; e++){
				if (rhythm[e]) printf("1");
				else printf("0");
			}
			printf(" \n");
			for (int f = 0; f < numSteps; f++){
				if (rhythm2[f]) printf("1");
				else printf("0");
			}
			printf(" \n");
			for (int f = 0; f < numSteps; f++){
				if (rhythm3[f]) printf("1");
				else printf("0");
			}
			printf(" \n");
			}
	}
	bool pulse = pgen.process( args.sampleTime );
	bool apulse = auxPgen.process( args.sampleTime );
	outputs[MAINOUT_OUTPUT].setVoltage(pulse ? 10.f : 0.f);
	outputs[AUXOUT_OUTPUT].setVoltage(apulse ? 10.f : 0.f);
	lights[CLOCKLED_LIGHT].setSmoothBrightness(inputs[CLOCK_INPUT].getVoltage(),args.sampleTime);
	lights[RESETLED_LIGHT].setSmoothBrightness(inputs[RESET_INPUT].getVoltage(),args.sampleTime);
	
	lights[MAINOUTLED_LIGHT].setSmoothBrightness(pulse,args.sampleTime);
	lights[AUXOUTLED_LIGHT].setSmoothBrightness(apulse,args.sampleTime);			
}

struct E325BCEWidget : ModuleWidget {
	E325BCEWidget(E325BCE* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/E325BCE.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<Trimpot>(mm2px(Vec(15.515, 27.651)), module, E325BCE::STEPS_PARAM));
		addParam(createParam<CKSS>(mm2px(Vec(23.085, 31.643)), module, E325BCE::NUMSTEPS_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(15.337, 47.728)), module, E325BCE::PULSES_PARAM));
		addParam(createParam<CKSS>(mm2px(Vec(23.264, 51.948)), module, E325BCE::MODE_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(15.383, 67.865)), module, E325BCE::OFFSET_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(6.403, 14.469)), module, E325BCE::CLOCK_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.174, 14.469)), module, E325BCE::RESET_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.129, 94.533)), module, E325BCE::STEPSCV_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.765, 94.533)), module, E325BCE::PULSESCV_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.638, 94.533)), module, E325BCE::OFFSETCV_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(6.269, 113.553)), module, E325BCE::AUXOUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.358, 113.553)), module, E325BCE::MAINOUT_OUTPUT));

		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(11.253, 10.291)), module, E325BCE::CLOCKLED_LIGHT));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(19.349, 10.303)), module, E325BCE::RESETLED_LIGHT));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(4.324, 106.843)), module, E325BCE::AUXOUTLED_LIGHT));
		addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(26.644, 106.886)), module, E325BCE::MAINOUTLED_LIGHT));
	}
};


Model* modelE325BCE = createModel<E325BCE, E325BCEWidget>("E325BCE");
