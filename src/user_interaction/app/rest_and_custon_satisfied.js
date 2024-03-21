

html_code = `
<button id="btn_rest" class="btn_end" value="satisfecho">Descansar</button>
          </div>`

// +++++++++++++++ boton de descanso +++++++++++++

//tiempos del descanso
let ini_time, end_time;
document.getElementById("btn_rest").onclick = () => {

    // se detienen las señales eeg
    stop_eeg();
    // se pausa el robot
    control_robot("pause");
    // pantalla completa
    openFullscreen();
    //tiempo ini
    ini_time = new Date();
    let container = document.getElementById("container_rest");
    // container.setAttribute("is_rest", "rest");
    container.style.visibility = "visible";
};

// +++++++++ boton de continuar despues del cansancio +++++++++++++
let btn_continuar = document.getElementById("btn_continuar");
btn_continuar.addEventListener("click", () => {
    play_gazebo();

    // se regresa la pantalla a su tamaño original
    closeFullscreen();

    get_dir().then((dir_user) => {


        //se obtiene el individuo actual
        load_object(`${dir_user}/actual_ind.txt`).then((actual_ind) => {
            let id_pareto = actual_ind.id_experiment;
            let num_ind = actual_ind.num_ind;
            let storage_pos = actual_ind.storage_pos;

            // se reinicia la simulación
            control_robot("unpause").then(() => {
            }).catch((e) => {
                console.error(e);
            });

            let container = document.getElementById("container_rest");
            // se inicial el flujo EEG
            play_eeg(id_pareto, num_ind, storage_pos).then(() => {

                load_object(`${dir_user}/history/histo_rest.txt`).then((his_r) => {
                    //oculta la ventana
                    container.style.visibility = "hidden";
                    // container.setAttribute("is_rest", "");
                    //reset del area de texto
                    document.getElementById("description_pause").value = "";

                    // id del individuo
                    let id_ind = `ind_${num_ind}`;

                    if (his_r[id_pareto] == undefined) his_r[id_pareto] = {};
                    if (his_r[id_pareto][id_ind] == undefined) his_r[id_pareto][id_ind] = {};

                    // motivo del descanso
                    reg["description"] = document.getElementById("description_pause").value;

                    //tiempo end
                    end_time = new Date();
                    //tiempo en milisegundos
                    reg["ini_time"] = ini_time.getTime();
                    reg["time"] = end_time.getTime() - ini_time.getTime();

                    his_r[id_pareto][id_ind][storage_pos] = reg;

                    // se actualiza el registro de descanso
                    COMM_FUNCT.save_object(his_r, `histo_rest.txt`, `${dir_user}/history`).catch((e) => {
                        console.error(e);
                    });
                });
            }).catch((e) => {
                //oculta la ventana
                container.style.visibility = "hidden";
                // container.setAttribute("is_rest", "");
                //reset del area de texto
                document.getElementById("description_pause").value = "";

                console.error(e);
            });

        });

    }).catch((e) => {
        console.error(e);
    });
});




// +++++++++++++++ customer satisfied score +++++++++++++

let score_ = document.getElementsByClassName("score");
for (let n_score = 0; n_score < score_.length; n_score++) {
    const elem = score_[n_score];

    elem.addEventListener("click", (emot) => {


        let score_val = emot.target.getAttribute("value");
        // console.log(score_val);

        get_dir().then((dir_user) => {
            COMM_FUNCT.save_object({ score: score_val }, "customer_satisfied_score.txt", dir_user).then(() => {
                //ventana de despedida
                document.body.innerHTML = txt;
            });
        });
    });
}

