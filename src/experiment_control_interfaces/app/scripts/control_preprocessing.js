//preprocessing init
function get_values_preprocessing() {
    return new Promise((resolve, reject) => {
        get_preprocessing_val().then((res) => {
            document.getElementById("do_erase_noise").checked = res.remove_environment_noise;
            document.getElementById("do_bandpass").checked = res.perform_bandpass;
            document.getElementById("do_detrend").checked = res.detrend;
            document.getElementById("do_save").checked = res.is_save;
            document.getElementById("do_reference_eeg").checked = res.reference_eeg;
            document.getElementById("do_reference_eeg_common").checked = res.reference_eeg_common;

            document.getElementById("freq_ini").value = res.frequency_ini;
            document.getElementById("freq_end").value = res.frequency_end;
            document.getElementById("filter_order").value = res.filter_order;

            resolve(res);
        }).catch(() => {
            reject();
        });
    });
}


function change_preprocessing_val() {
    let bool_change = [document.getElementById("do_erase_noise"), document.getElementById("do_bandpass"),
    document.getElementById("do_detrend"), document.getElementById("do_save"), document.getElementById("do_reference_eeg"),
    document.getElementById("do_reference_eeg_common")];

    //cambiando los boleanos
    bool_change.forEach(elem => {
        elem.addEventListener("click", () => {

            set_preprocessing_val(0, 0, 0, bool_change[0].checked, bool_change[1].checked, bool_change[2].checked, bool_change[3].checked, bool_change[4].checked, bool_change[5].checked).then(() => {
                get_values_preprocessing().then(() => {
                    show_blink(elem, color_success);
                });
            });
        });
    });

    //cambiando los valores numéricos
    let value_change = [document.getElementById("freq_ini"),
    document.getElementById("freq_end"),
    document.getElementById("filter_order")];
    value_change.forEach(elem => {
        elem.addEventListener("keypress", (e) => {
            if (e.key != "Enter") return;

            set_preprocessing_val(value_change[0].value, value_change[1].value, value_change[2].value, bool_change[0].checked, bool_change[1].checked, bool_change[2].checked, bool_change[3].checked).then(() => {
                get_values_preprocessing().then(() => {
                    show_blink(elem, color_success);
                });
            });
        });
    });
}


// ----------------------
change_preprocessing_val();
get_values_preprocessing();
