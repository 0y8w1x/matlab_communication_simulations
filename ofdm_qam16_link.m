% qam properties
modulation_order = 16;
bits_per_symbol = log2(modulation_order);
% signal to noise ratio in db
snr = 15;

% spectrum analyzer
scope = dsp.SpectrumAnalyzer("NumInputPorts", 2, "ViewType", "Spectrum and spectrogram",...
    "SampleRate", 10000, "FrequencySpan", "Span and center frequency", "Span", 10e3);

% ofdm modulator/demodulator
ofdm_modulator = comm.OFDMModulator("FFTLength", 64, ...
    "NumGuardBandCarriers", [6; 5], "InsertDCNull", true, "PilotInputPort", 1,...
    "PilotCarrierIndices", [12; 26; 40; 54], "CyclicPrefixLength", 16);
ofdm_demodulator = comm.OFDMDemodulator(ofdm_modulator);
ofdm_dims = info(ofdm_modulator);
showResourceMapping(ofdm_modulator);

% convolutional code structure
constraint_length = 3;
trellis_structure = poly2trellis(constraint_length, [6, 7]);

% number of ofdm symbols
number_of_frames = 10000;
% size of one ofdm symbol
frame_size = 96;
% number of bits to be transmitted
num_bits = number_of_frames * frame_size;
% data to be transmitted as bits
data = randi([0 1], num_bits, 1);

% statistics
worst_ber = 0;
ber_summation = 0;
cnt = 1;

for i = 1:(num_bits/frame_size)
    data_frame = data(cnt:cnt+frame_size-1);
    cnt = cnt + frame_size;
    
    % convolutional encode the data frame with viterbi
    encoded_data = convenc(data_frame, trellis_structure);
    
    % interleave data
    matrix_data = reshape(encoded_data, [], bits_per_symbol);
    % note: last 2 numbers in matintrlv have to multiply to be equal to bits_per_symbol
    interleaved_data = matintrlv(matrix_data', 2, 2)';
    
    % qam modulation
    symbol_data = bi2de(interleaved_data);
    qam_modulated_signal = qammod(symbol_data, modulation_order, "UnitAveragePower", 1);
    
    % ofdm modulation
    pilot_symbols_in = complex(rand(ofdm_dims.PilotInputSize),rand(ofdm_dims.PilotInputSize));
    ofdm_modulated_signal = ofdm_modulator(qam_modulated_signal, pilot_symbols_in);
    
    % channel impairment
    channel_impaired_signal = awgn(ofdm_modulated_signal, snr, "measured");
    % visualize frequency spectrum
    scope(ofdm_modulated_signal, channel_impaired_signal);
    
    % ofdm demodulation
    [ofdm_demodulated_signal, pilot_symbols_out] = ofdm_demodulator(channel_impaired_signal);
    
    % qam demodulation
    qam_demodulated_data = qamdemod(ofdm_demodulated_signal, modulation_order, "UnitAveragePower", 1);
    binary_data = de2bi(qam_demodulated_data');
    
    % deinterleave data
    deinterleaved_data = matdeintrlv(binary_data', 2, 2)';
    deinterleaved_data = deinterleaved_data(:);
    
    % convolutional decode data frame with viterbi
    decoded_data = vitdec(deinterleaved_data, trellis_structure, 5, "trunc", "hard");
    
    % calculate BER
    error_rate = comm.ErrorRate();
    error = error_rate(data_frame, decoded_data);
    ber_summation = ber_summation + error(1);
    if error(1) > worst_ber
        worst_ber = error(1);
    end
end

disp("average BER: " + round(ber_summation/(num_bits/frame_size)*100, 2) + "%");
disp("worst BER: " + round(worst_ber*100, 2) + "%");