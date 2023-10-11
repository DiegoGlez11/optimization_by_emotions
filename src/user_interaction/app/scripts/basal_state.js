
function start_basal_state(id_pareto, num_ind, storage_pos, time_basal = 2000) {
    return new Promise((resolve, reject) => {
        // if (EMOTIONS_CAPTURE_MODE == "sam") {
        //     resolve();
        //     return;
        // }

        let win_msg_ = document.getElementById("container_msg");
        win_msg_.innerHTML = "<div id='cyton'><h1>Adquiriendo estado basal</h1></div>\n\n";
        win_msg_.style.visibility = "visible";

        let err = function (e) {
            console.error("Error en la captura del estado basal\n\n", e);
            win_msg_.style.visibility = "hidden";
            reject(e);
        };


        // se inicia el flujo eeg
        play_eeg("optimized_basal").then(() => {

            setTimeout(() => {

                stop_eeg().then((res) => {
                    win_msg_.style.visibility = "hidden";

                    // dirección del usuario
                    get_dir().then((dir_user) => {
                        if (storage_pos == undefined) resolve();

                        // se carga el historial basal
                        let path = `${dir_user}/history/histo_basal_state_dict.txt`;
                        load_object(path).then((histo_basal) => {

                            // if (histo_basal.length == undefined) histo_basal = [];
                            // // se actualiza el registro del estado basal
                            // histo_basal.push(res.id_eeg_signal);

                            let id_ind = `ind_${num_ind}`;
                            if (histo_basal[id_pareto] == undefined) histo_basal[id_pareto] = {};
                            if (histo_basal[id_pareto][id_ind] == undefined) histo_basal[id_pareto][id_ind] = {};

                            histo_basal[id_pareto][id_ind][storage_pos] = res.id_eeg_signal;

                            save_object(histo_basal, "histo_basal_state_dict.txt", `${dir_user}/history`).then(() => {

                                resolve();
                            }).catch((e) => {
                                err(e);
                            });
                        });
                    }).catch((e) => {
                        err(e);
                    });
                }).catch((e) => {
                    err(e);
                });
            }, time_basal);
        }).catch((e) => {
            err(e);
        });
    });
}