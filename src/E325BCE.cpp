#include "plugin.hpp"
#include "dsp/digital.hpp"
#define TRIG_TIME 1e-3f
#define DEBUG_PRINT 1

#define NORMAL 1
#define STEPS_16 1
struct E325BCE : Module {
	enum ParamId {
		STEPS_PARAM,
		NumberOfSteps_PARAM,
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
	dsp::PulseGenerator PulseGenerator;
	dsp::PulseGenerator AuxOutPulseGenerator;
	dsp::SchmittTrigger clockDetector;
	dsp::SchmittTrigger resetDetector;
	int StepNumber = 0;
	int pattern1[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
	int pattern2[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
    int pattern3[32] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};	
	int lastNumberOfSteps = 0;
	int NumberOfSteps = 16;

	E325BCE() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam (STEPS_PARAM,         1.f, 16.f, 16.f, "Steps");
		configParam (NumberOfSteps_PARAM, 0.f,  1.f,  0.f, "16/32");
		configParam (PULSES_PARAM,        0.f, 16.f, 16.f, "Pulses");
		configParam (MODE_PARAM,          0.f,  1.f,  1.f, "Norm/XOR!!!");
		configParam (OFFSET_PARAM,        0.f, 16.f,  0.f, "Offset");
		configInput (CLOCK_INPUT,    "Clock In");
		configInput (RESET_INPUT,    "Reset In");
		configInput (STEPSCV_INPUT,  "Steps CV");
		configInput (PULSESCV_INPUT, "Pulses CV");
		configInput (OFFSETCV_INPUT, "Offset CV");
		configOutput(AUXOUT_OUTPUT,  "Aux Pulse");
		configOutput(MAINOUT_OUTPUT, "Pulse");
		paramQuantities[STEPS_PARAM] -> snapEnabled = true;
		paramQuantities[PULSES_PARAM]-> snapEnabled = true;
		paramQuantities[OFFSET_PARAM]-> snapEnabled = true;
	}

	void process(const ProcessArgs& args) override;

	void GenerateEuclidPattern(int *patternArray, int steps, int pulses){
		
		for (int i = 0; i<steps; i++) patternArray[i] = 0;
		int bucket = 0;
		for (int i=0; i < steps; i++){
			bucket += pulses;
		    if (bucket >= steps) {
		    	bucket -= steps;
		    	patternArray[i] = 1;
		    } else patternArray[i] = 0;
		}
		int tempArray[32] = {0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 ,0,0,0,0, 0,0,0,0};
		tempArray[0] = pattern1[NumberOfSteps-1];
		for(int i = 1; i<NumberOfSteps; i++) tempArray[i] = pattern1[i-1];
		for(int i = 0; i<NumberOfSteps; i++) pattern1[i] = tempArray[i];
	}				
};

void E325BCE::process(const ProcessArgs &args) {

	// Get the value of the Number Of Steps Switch
	int NumberOfStepsSwitchValue = (int)params[NumberOfSteps_PARAM].getValue() ;
	// Get the value of the Mode Switch
	int ModeSwitchValue = (int)params[MODE_PARAM].getValue();
	
	// Get the steps knob amount and steps CV and add them up for the final value
	// If the final value is more than the max, set it to the max
	int StepsKnobValue = (int)params[STEPS_PARAM].getValue(); 
	int StepsCVValue = (int)inputs[STEPSCV_INPUT].getVoltage();
	int StepsValue = StepsKnobValue + StepsCVValue;
	if (StepsValue > NumberOfSteps) StepsValue = NumberOfSteps;

	// Get the pulses knob amount and pulses CV and add them up for the final value
	// If the final value is more than the max, set it to the max
	int PulsesKnobValue = (int)params[PULSES_PARAM].getValue();
	int PulsesCVValue = (int)inputs[PULSESCV_INPUT].getVoltage();
	int PulsesValue = PulsesKnobValue + PulsesCVValue;
	if (PulsesValue > NumberOfSteps) PulsesValue = NumberOfSteps;
	
	// Get the offset knob amount and offset CV and add them up for the final value
	// If the final value is more than the max, set it to the max
	int OffsetKnobValue = (int)params[OFFSET_PARAM].getValue();
	int OffsetCVValue = (int)inputs[OFFSETCV_INPUT].getVoltage();
	int OffsetValue = OffsetKnobValue + OffsetCVValue;
	if (OffsetValue > NumberOfSteps) OffsetValue = NumberOfSteps;

	// if the 16/32 switch has changed, then adjust knob parameters
	if (NumberOfStepsSwitchValue != lastNumberOfSteps) {
		lastNumberOfSteps = NumberOfStepsSwitchValue;
		if ((int)params[NumberOfSteps_PARAM].getValue() != STEPS_16) {
			configParam(STEPS_PARAM, 1.f, 16.f, 16.f, "Steps");
			configParam(PULSES_PARAM, 0.f, 16.f, 16.f, "Pulses");
			configParam(OFFSET_PARAM, 0.f, 16.f, 0.f, "Offset");
			NumberOfSteps = 16;
		}
		else {
			configParam(STEPS_PARAM, 1.f, 32.f, 32.f, "Steps");
			configParam(PULSES_PARAM, 0.f, 32.f, 32.f, "Pulses");
			configParam(OFFSET_PARAM, 0.f, 32.f, 0.f, "Offset");
			NumberOfSteps = 32;
		}
		paramQuantities[STEPS_PARAM]->snapEnabled = true;
		paramQuantities[PULSES_PARAM]->snapEnabled = true;
		paramQuantities[OFFSET_PARAM]->snapEnabled = true;	
	}

	if (ModeSwitchValue == 1) 
	{
		GenerateEuclidPattern ( pattern1, StepsValue, PulsesValue);
	}
	else
	{
		GenerateEuclidPattern (pattern1, NumberOfSteps, StepsValue);
		GenerateEuclidPattern (pattern2, NumberOfSteps, PulsesValue);

		for (int w = 0; w<NumberOfSteps; w++){
			pattern3[w] = (pattern1[w] ^ pattern2[w]);
		}			
	}
	 
	if (resetDetector.process(inputs[RESET_INPUT].getVoltage())) StepNumber = 0;

	if (clockDetector.process(inputs[CLOCK_INPUT].getVoltage())) {

			int offsetVal = StepNumber + OffsetValue;
					
			if (ModeSwitchValue == NORMAL) 
			{
				if (pattern1[offsetVal]) PulseGenerator.trigger(TRIG_TIME);
				else               AuxOutPulseGenerator.trigger(TRIG_TIME);				
			}
			else
			{
				if (pattern3[offsetVal]) PulseGenerator.trigger(TRIG_TIME);
				else               AuxOutPulseGenerator.trigger(TRIG_TIME);

			}
			StepNumber++;

			if (ModeSwitchValue == NORMAL) 
			{
				if (StepNumber > StepsValue-1) StepNumber = 0;
			}
			else
			{
				if(StepNumber > NumberOfSteps-1) StepNumber=0;
			}
			
			printf("=-=-=-=-=-=-=-=- =-=-\n");
			if (DEBUG_PRINT) {
			printf("Step: %d\n", StepNumber);
			if (ModeSwitchValue == NORMAL) 
			{
				for (int e = 0; e < (int)(params[STEPS_PARAM].getValue()); e++){
					if (pattern1[e]) printf("1");
					else printf("0");
				}
			} 
			else
			{
				for (int e = 0; e < NumberOfSteps; e++){
					if (pattern1[e]) printf("1");
					else printf("0");
				}
			}
			printf(" \n");
			for (int f = 0; f < NumberOfSteps; f++){
				if (pattern2[f]) printf("1");
				else printf("0");
			}
			printf(" \n");
			for (int f = 0; f < NumberOfSteps; f++){
				int offsetVal2 = f + OffsetKnobValue+(int)inputs[OFFSETCV_INPUT].getVoltage();
				if (offsetVal2 > NumberOfSteps-1) 
				{
					offsetVal2 = offsetVal2 - NumberOfSteps-1;
				}
				if (offsetVal2 > NumberOfSteps-1) 
				{
					offsetVal2 = offsetVal2 - NumberOfSteps-1;
				}
				if (pattern3[offsetVal2]) printf("1");
				else printf("0");
			}
			printf(" \n");
			}
	}
	bool pulse = PulseGenerator.process( args.sampleTime );
	bool apulse = AuxOutPulseGenerator.process( args.sampleTime );
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
		addParam(createParam<CKSS>(mm2px(Vec(23.085, 31.643)), module, E325BCE::NumberOfSteps_PARAM));
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
