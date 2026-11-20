/**
 * @file
 * Model-table parsing checks using small malformed/absent-operation fixtures.
 * Reject invalid offsets and custom-op names appearing only in unrelated data.
 * An optional model path enables a real file check; no accelerator is opened.
 */

#include "../src/model_check.h"
#include <iostream> // Console progress, usage information, and actionable error reporting.
int main(int argc, char **argv) {
    try {
        using namespace tabletop;
        if (has_edge_tpu_operator({1, 2, 3, 4, 5, 6, 7, 8})) // Arbitrary eight-byte data must not pass the TFL3 identifier and operator-table check.
            throw std::runtime_error("Non-model accepted");
        bool rejected = false; // Record whether a deliberately invalid root offset triggers checked parsing failure.
        try {
            has_edge_tpu_operator({255, 255, 255, 255, 'T', 'F', 'L', '3'}); // Supply a correct identifier with an out-of-bounds root offset to exercise bounds checking.
        } catch (const std::exception &) {
            rejected = true; // The malformed-offset fixture was rejected through the expected exception path.
        }
        if (!rejected) // Fail if malformed offsets were accepted or silently treated as safe model data.
            throw std::runtime_error("Bad offset accepted");
        std::vector<unsigned char> fake{12, 0, 0, 0, 'T', 'F', 'L', '3', 4, 0, 4, 0, 4, 0, 0, 0}; // A table with no operator codes must not become a TPU model just because the byte string appears in trailing data or metadata. Build a tiny structurally readable root table with no operator-code entries.
        std::string token = "edgetpu-custom-op"; // Use the exact accelerator operator name as unrelated trailing data in the negative fixture.
        fake.insert(fake.end(), token.begin(), token.end()); // Append the text outside any declared operator table to defeat naive substring-based validation.
        if (has_edge_tpu_operator(fake)) // Require structural operator discovery, not a raw byte-string match anywhere in the file.
            throw std::runtime_error("Metadata token accepted as operator");
        if (argc > 1) { // Optionally validate a real downloaded detector in addition to the built-in negative fixtures.
            auto data = read_detector_model(argv[1]); // Read and structurally inspect the explicitly supplied model without opening an accelerator.
            if (data.empty()) // A successful real-model check must return owned nonempty bytes.
                throw std::runtime_error("Model empty");
            std::cout << "Real compiled detector validated: " << data.size() << " bytes\n";
        }
        std::cout << "Model operator-table checks passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
