clear;
clc;

% qam properties
modulation_order = 16;
bits_per_symbol = log2(modulation_order);

% rrc filter properties
rolloff = 0.3;
% span in symbols
span = 6;
% samples per symbol
sps = 8;

% number of bits to be transmitted
num_bits = 10000;
% resulting number of symbols
num_symbols = num_bits/bits_per_symbol;

%% Data
% data to be transmitted as bits
data = randi([0 1], num_bits, 1);
symbol_data = bi2de(reshape(data, num_symbols, bits_per_symbol));

%% QAM-16 Modulator
% qam modulation
qam_modulated_signal = qammod(symbol_data, modulation_order, "gray", "UnitAveragePower", 1);

%% Transmit RRC Filter
% create rrc filter
rrc_transmit_filter = comm.RaisedCosineTransmitFilter("Shape", "Square root", ...
    "RolloffFactor", rolloff, "FilterSpanInSymbols", span, ...
    "OutputSamplesPerSymbol", sps);
rrc_transmit_signal = rrc_transmit_filter(qam_modulated_signal);

%% Power Amplifier
% back-off level in db
power_amplifier_backoff_level = 30;
% root raised cosine compensation in db
rrc_compensation = mag2db(0.38);
% linear PA gain
linear_pa_gain = 18;
% difference between linear gain and small signal gain
alpha = mag2db(2.1587);
% rrc filter gain
rcc_gain = pow2db(sps);
% input gain of PA in db
input_gain = -power_amplifier_backoff_level - rrc_compensation - rcc_gain;
% output gain of PA in db
output_gain = power_amplifier_backoff_level + rrc_compensation - alpha + linear_pa_gain;
% create power amplifier
power_amplifier = comm.MemorylessNonlinearity("Method","Saleh model", "InputScaling", input_gain,...
    "AMAMParameters", [2.1587 1.1517], "AMPMParameters", [4.0330 9.1040],...
    "OutputScaling", output_gain);

amplified_signal = power_amplifier(rrc_transmit_signal);

%% Transmit Antenna
% transmitter antenna gain
transmit_antenna_gain = db2pow(5);
transmit_antenna_signal = amplified_signal .* transmit_antenna_gain;

%% Downlink
% altitude in m
satellite_altitude = 35600e3;
% carrier frequency in Hz
frequency = 10e9;
% wavelength
wavelength = physconst("LightSpeed")/frequency*1e9;
% free space loss in db
free_space_loss = fspl(satellite_altitude, wavelength);
free_space_gain = 1/db2pow(free_space_loss);
path_loss_signal = transmit_antenna_signal * free_space_gain;

% phase and frequency impairments
phase_frequency_offset = comm.PhaseFrequencyOffset("PhaseOffset", 0, "FrequencyOffset", 0);
doppler_impaired_signal = phase_frequency_offset(path_loss_signal);

%% Receive Antenna
% receiver antenna gain
receive_antenna_gain = db2pow(5);
receive_antenna_signal = doppler_impaired_signal .* receive_antenna_gain;

% thermal noise
thermal_noise = comm.ThermalNoise("NoiseTemperature", 20);
thermal_noise_signal = thermal_noise(receive_antenna_signal);

% phase noise
phase_noise = comm.PhaseNoise("Level", -100, "FrequencyOffset", 100);
phase_noise_signal = phase_noise(thermal_noise_signal);

% iq-imbalance
amplitude_imbalance = 0;
phase_imbalance = 0;
i_dc_offset = 0;
q_dc_offset = 0;
iq_imbalance_signal = iqimbal(phase_noise_signal, amplitude_imbalance, phase_imbalance);
iq_offset_signal = iq_imbalance_signal + complex(i_dc_offset, q_dc_offset);

%% Low Noise Amplifier
lna_gain = db2pow(10);
lna_signal = iq_offset_signal .* lna_gain;

%% Receive RRC Filter
rrc_receive_filter = comm.RaisedCosineReceiveFilter("Shape", "Square root", ...
    "RolloffFactor", rolloff, "FilterSpanInSymbols", span, ...
    "InputSamplesPerSymbol", sps, "DecimationFactor", sps, "Gain", 1/sqrt(sps));
rrc_receive_signal = rrc_receive_filter(lna_signal);

%% DC Blocker
dc_blocker = dsp.DCBlocker("Algorithm", "IIR", "NormalizedBandwidth", 0.0005, ...
    "Order", 3);
dc_blocked_signal = dc_blocker(rrc_receive_signal);

%% Automatic Gain Control
automatic_gain_control = comm.AGC("DesiredOutputPower", 1, "AveragingLength",...
    256, "MaxPowerGain", 400);
agc_signal = automatic_gain_control(dc_blocked_signal);

%% Impairment Corrections
% iq imbalance correction
iq_imbalance_compensator = comm.IQImbalanceCompensator();
iq_imbalance_corrected_signal = iq_imbalance_compensator(agc_signal);

% doppler correction with carrier synchronizer
carrier_synchronizer = comm.CarrierSynchronizer("Modulation", "QAM", "SamplesPerSymbol", 2);
carrier_synchronized_signal = carrier_synchronizer(iq_imbalance_corrected_signal);

%% QAM-16 Demodulator
qam_demodulated_data = qamdemod(carrier_synchronized_signal, modulation_order, "gray", "UnitAveragePower", 1);
% received bit data
received_data = reshape(de2bi(qam_demodulated_data), num_bits, 1);

% calculate BER
error_rate = comm.ErrorRate("ReceiveDelay", 6);
error = error_rate(data, received_data);

disp("BER: " + round(error(1)*100, 1) + "%");

% constellation diagram of filtered and amplified signal
constellation_diagram = comm.ConstellationDiagram(2, "SamplesPerSymbol", sps, ...
    "SymbolsToDisplaySource", "Property", "SymbolsToDisplay", num_bits, ...
    "ReferenceConstellation", qam_modulated_signal, 'ShowLegend', true, ...
    "XLimits", [-5 5],"YLimits", [-5 5], "ChannelNames", ["TX_rrc", "TX_amplified"]);
constellation_diagram(rrc_transmit_signal, amplified_signal);

% spectrum analyzer
spectrum_analyzer = dsp.SpectrumAnalyzer("NumInputPorts", 2, "ViewType", "Spectrum and spectrogram",...
    "SampleRate", 10000, "FrequencySpan", "Span and center frequency", "Span", 10e3);
