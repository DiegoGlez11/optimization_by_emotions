let eeg_channels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
let eeg_names = ["Fp1", "Fp2", "C3", "C4", "P7", "P8", "O1", "O2", "F7", "F8", "F3", "F4", "T7", "T8", "P3", "P4"];


function graph_view(txt, color, win_size, slide) {
    this.slice_window = slide;
    this.window_size = win_size;

    //opciones de la gráfica
    let options = {

        pointRadius: 0,
        borderWidth: 1,
        pointBackgroundColor: "blue",
        // animation: false,
        maintainAspectRatio: false,
        responsive: true,
        scales: {
            y: {
                grid: {
                    color: 'blue',
                    borderColor: "red",
                },
            },
            x: {
                ticks: {
                    display: false,
                },
                // min: 0,
                // max: 625,
            }
        },
        changeScale: true,

        plugins: {
            zoom: {
                pan: {
                    enabled: true,
                    mode: "y",
                },
                zoom: {
                    wheel: {
                        enabled: false,
                    },
                    drag: {
                        enabled: false,
                        modifierKey: "ctrl",
                    },
                    pinch: {
                        enabled: false,
                    },
                    mode: 'y',

                }
            },
            legend: {
                display: false,
            },
            tooltip: {
                enabled: false,
            },
        },
    };

    //configuración de la gráfica
    let config = {
        type: "line",
        data: {
            datasets: [{
                label: txt,
                data: [],
                lineTension: 0,
                backgroundColor: "rgba(0,0,0,0)",
                borderColor: color,

            },],
        },
        options: options,
    };

    //contenedor de la gráfica
    this.container = document.createElement("div");
    this.container.setAttribute("class", "container_graph");

    //lugar donde se muestra la gráfica
    this.graph_html = document.createElement("canvas");
    this.graph_html.setAttribute("class", "graph");
    this.container.appendChild(this.graph_html);

    //contenedor del canal
    this.container_channel = document.createElement("div");
    this.container_channel.setAttribute("class", "container_channel")

    //indicador de canal
    let ind_chan = document.createElement("div");
    ind_chan.innerHTML = txt;
    this.container_channel.appendChild(ind_chan);

    //contenedor botones
    let cont_btn = document.createElement("div");
    cont_btn.setAttribute("class", "container_btn_channel");

    //botones de limites de la gráfica
    let btn_up = document.createElement("div");
    btn_up.setAttribute("class", "btn_channel btn_up");
    cont_btn.appendChild(btn_up);

    let btn_down = document.createElement("div");
    btn_down.setAttribute("class", "btn_channel btn_down");
    cont_btn.appendChild(btn_down);

    // botones para mover la gráfica en el eje x
    this.btn_left = document.createElement("div");
    this.btn_left.setAttribute("class", "btn_channel btn_left");
    cont_btn.appendChild(this.btn_left);

    this.btn_right = document.createElement("div");
    this.btn_right.setAttribute("class", "btn_channel btn_right");
    cont_btn.appendChild(this.btn_right);

    this.container_channel.appendChild(cont_btn);
    this.container_channel.appendChild(this.container);

    //creación de la gráfica
    let ctx = this.graph_html.getContext("2d");
    this.graph = new Chart(ctx, config);
    this.graph.height = 200;

    //limites de los datos
    this.get_limits = function () {
        let min_ = Math.min(...this.graph.data.datasets[0].data);
        let max_ = Math.max(...this.graph.data.datasets[0].data);
        return { min: min_, max: max_ };
    };

    //eventos de los botones
    let size_range = 50;
    btn_up.addEventListener("click", () => {

        let max = this.graph.config.options.scales.y.max;
        let min = this.graph.config.options.scales.y.min;
        if (max == undefined || min == undefined) {
            min = Math.min(...this.graph.data.datasets[0].data);
            max = Math.max(...this.graph.data.datasets[0].data);
        }

        this.graph.config.options.scales.y.max = max + size_range;
        this.graph.config.options.scales.y.min = min - size_range;
        this.graph.update();
    });

    btn_down.addEventListener("click", () => {
        let max = this.graph.config.options.scales.y.max;
        let min = this.graph.config.options.scales.y.min;
        if (max == undefined || min == undefined) {
            min = Math.min(...this.graph.data.datasets[0].data);
            max = Math.max(...this.graph.data.datasets[0].data);
        }

        this.graph.config.options.scales.y.max = max - size_range;
        this.graph.config.options.scales.y.min = min + size_range;
        this.graph.update();
    });

    return this;
}


function new_graph(parent, topic_name, topic_type) {
    //valores iniciales de los controles
    let window_size = 5 * 125; //tamaño de las muestras en la gráfica
    let slice_window = 80; //cada cuantas muestras se genera una ventana (vista de tamaño window_size)
    let num_channels = 16;
    let graphs_aux = this;


    // se usa para crear campos numéricos para cambiar los valores de los parámetros
    let number_option = function (text, value_in) {
        let cont_op = document.createElement("div");
        cont_op.setAttribute("class", "container_opt");

        let label = document.createElement("label");
        label.setAttribute("for", "ctrl_opt");
        label.innerHTML = text;

        let input_ctr = document.createElement("input");
        input_ctr.setAttribute("class", "ctrl_opt");
        input_ctr.setAttribute("type", "number");
        input_ctr.setAttribute("min", 1);
        input_ctr.setAttribute("value", value_in);

        cont_op.appendChild(label);
        cont_op.appendChild(input_ctr);

        //eventos
        input_ctr.addEventListener("keypress", (e) => {
            if (e.key != "Enter") return;

            let is_reset = false;
            if (text == "window_size") {
                is_reset = true;
                window_size = parseInt(input_ctr.value);
            }
            if (text == "slice_window") {
                slice_window = parseInt(input_ctr.value);
            }

            if (is_reset) {
                let l = graphs_aux.graphs.length;
                for (let gi = 0; gi < l; gi++) {
                    graphs_aux.graphs[gi].update_window_size();
                }
            }

            show_blink(input_ctr, color_success);
        });

        return cont_op;
    };

    if (parent == undefined) {
        console.error("El contenedor de la gráfica esta vació");
        return;
    }
    if (topic_name == undefined || topic_type == undefined) {
        console.error("Error al conectar con el topico, faltan parámetros");
        return;
    }

    //controles de la gráfica
    let controls = document.createElement("div");
    controls.setAttribute("class", "container_controls");
    controls.appendChild(number_option("window_size", window_size));
    controls.appendChild(number_option("slice_window", slice_window));
    parent.appendChild(controls);

    //colores de los canales
    let colors = ["grey", "rebeccapurple", "blue", "green", "rgb(185, 174, 22)", "orange", "red", "rgb(126, 27, 27)", "grey", "rebeccapurple", "blue", "green", "rgb(185, 174, 22)", "orange", "red", "rgb(126, 27, 27)"];
    //datos de los canales
    let eeg_data = [];
    this.graphs = [];//gráficas
    this.labels = [];

    // resetea los canales EEG
    this.create_channels = function () {
        // nueva gráfica
        graphs_aux.graphs = [];
        //datos de los canales
        eeg_data = [];

        // ceros de inicialización
        let zero = [];
        graphs_aux.labels = [];
        for (let i = 0; i < window_size; i++) {
            zero.push(0);
            graphs_aux.labels.push(i);
        }

        //se generan las gráficas de cada canal
        for (let chan = 0; chan < num_channels; chan++) {
            let g = new graph_view(`canal ${eeg_channels[chan]} nombre: ${eeg_names[chan]}`, colors[chan], window_size, slice_window);
            // se guarda el canal
            graphs_aux.graphs.push(g);
            parent.appendChild(g.container_channel);
            // datos iniciales
            eeg_data.push(Object.assign([], zero));
            g.graph.data.datasets[0].data = Object.assign([], zero);
            g.graph.config.data.labels = graphs_aux.labels;
            g.graph.update("none");
        }
    }

    let clean_signals = document.getElementById("clean_signals");
    clean_signals.onclick = () => {
        // ceros de inicialización
        let zero = [];
        graphs_aux.labels = [];
        for (let i = 0; i < window_size; i++) {
            zero.push(0);
            graphs_aux.labels.push(i);
        }
        //datos de los canales
        eeg_data = [];

        for (let n_g = 0; n_g < graphs_aux.graphs.length; n_g++) {
            let g = graphs_aux.graphs[n_g];

            // datos iniciales
            eeg_data.push(Object.assign([], zero));
            g.graph.data.datasets[0].data = Object.assign([], zero);
            g.graph.config.data.labels = graphs_aux.labels;
            // g.graph.update("none");
            g.reset_view();
        }
    }

    // se crean las gráficas de los canales
    this.create_channels();

    // let actual_block;
    let time_capturing;
    let time_update = [];
    //mutex
    let mutex = new Mutex();
    let wait_time = 90;
    this.stop_update = false;
    // metadatos de los bloques a mostrar
    let view_block = [];

    //actualización de la gráfica para la forma online
    async function run_update() {
        let ini_pos;
        let end_pos = window_size - slice_window;

        while (!graphs_aux.stop_update) {
            //Tamaños
            let size_signal = eeg_data[0].length;
            let size_time = time_update.length;

            //no hay datos
            if (size_signal == 0 || size_time == 0) {
                await sleep(wait_time);
                continue;
            }

            // se extrae la info del bloque EEG a mostrar
            let actual_block = time_update.pop();

            // historial de vista
            view_block.push(actual_block);
            // console.log(view_block);

            // se muestra el bloque EEG
            // for (let n_win = 0; n_win < actual_block.num_windows; n_win++) {
            while (true) {
                // //pos final
                // end_pos = actual_block.ini + (n_win + 1) * slice_window;
                // if (end_pos > actual_block.end) end_pos = actual_block.end;
                // // pos inicial
                // ini_pos = end_pos - window_size;

                // // console.log("ini:", ini_pos, "end:", end_pos, "size_signal", size_signal);

                // //labels del eje x
                // // graphs_aux.labels = [];
                // let l = 0;
                // for (let k = ini_pos; k < end_pos; k++) {
                //     graphs_aux.labels[l] = k;
                //     l++;
                // }
                let is_end = false;
                // //para cada canal se extraen los datos a mostrar
                for (let n_chan = 0; n_chan < num_channels; n_chan++) {
                    //     //datos de las señales a mostrar en la gráfica
                    //     graphs_aux.graphs[n_chan].graph.data.datasets[0]["data"] = eeg_data[n_chan].slice(ini_pos, end_pos);
                    //     graphs_aux.graphs[n_chan].graph.config.data.labels = graphs_aux.labels;
                    //     graphs_aux.graphs[n_chan].graph.update("none");
                    is_end = graphs_aux.graphs[n_chan].move_view("right");
                }

                await sleep(actual_block.time);
                if (is_end) break;
            }

            let threshold_view = parseInt(document.getElementById("visual_num").value);
            //eliminación de bloques
            if (view_block.length > threshold_view) {
                console.log("erase ");
                //sincronización
                mutex.runExclusive(function () {
                    let erase_blocks = view_block.splice(view_block.length - threshold_view);
                    let size_erase = erase_blocks[0].end - window_size;

                    //se eliminan los datos 
                    for (let n_chan = 0; n_chan < num_channels; n_chan++) {
                        eeg_data[n_chan].splice(0, size_erase);
                    }
                    // se actualizan los rangos
                    for (let n_t = 0; n_t < time_update.length; n_t++) {
                        time_update[n_t].ini -= size_erase;
                        time_update[n_t].end -= size_erase;
                    }
                });

            }
        }
        graphs_aux.stop_update = false;
        graphs_aux.is_run = false;
        console.log("\n\n\n\n\n\nexit.....");
    }
    // run_update();

    this.is_run = false;
    this.run_read_signal = function () {
        mutex.runExclusive(function () {
            if (!graphs_aux.is_run) {
                graphs_aux.is_run = true;
                graphs_aux.stop_update = false;
                run_update();
            }
        });
    }

    // +++++++++++++++++++++++++++++++++++++
    //eventos de botones izquierdo y derecho
    // +++++++++++++++++++++++++++++++++++++
    let size_g = graphs_aux.graphs.length;
    for (let num_graph = 0; num_graph < size_g; num_graph++) {
        let right = graphs_aux.graphs[num_graph].btn_right;
        let left = graphs_aux.graphs[num_graph].btn_left;

        let end_pos = window_size - slice_window;
        let ini_pos;

        let move_view = (type) => {
            let size_d = eeg_data[num_graph].length;
            let is_end = false;

            if (type == "left") {
                end_pos -= slice_window;
                if (end_pos < window_size) {
                    end_pos = window_size;
                    ini_pos = 0;
                    is_end = true;
                }
            }
            if (type == "right") {
                end_pos += slice_window;
                if (end_pos > size_d) {
                    end_pos = size_d;
                    ini_pos = end_pos - window_size;
                    is_end = true;
                }
            }

            ini_pos = end_pos - window_size;

            // console.log("size_d", size_d);
            // console.log("ini_pos", ini_pos, "end_pos", end_pos);

            graphs_aux.graphs[num_graph].graph.data.datasets[0].data = eeg_data[num_graph].slice(ini_pos, end_pos);
            // etiquetas   
            let labels_x = [];
            for (let i = ini_pos; i < end_pos; i++) labels_x.push(i);
            graphs_aux.graphs[num_graph].graph.config.data.labels = labels_x;
            graphs_aux.graphs[num_graph].graph.update("none");

            // console.log(ini_pos, end_pos, size_d);
            return is_end;
        }

        let reset_view = () => {
            end_pos = window_size;
            ini_pos = 0;

            graphs_aux.graphs[num_graph].graph.data.datasets[0].data = eeg_data[num_graph].slice(ini_pos, end_pos);
            // etiquetas   
            let labels_x = [];
            for (let i = ini_pos; i < end_pos; i++) labels_x.push(i);
            graphs_aux.graphs[num_graph].graph.config.data.labels = labels_x;
            graphs_aux.graphs[num_graph].graph.update("none");
        }
        graphs_aux.graphs[num_graph]["reset_view"] = reset_view;

        let update_window_size = () => {
            end_pos = ini_pos + window_size;

            graphs_aux.graphs[num_graph].graph.data.datasets[0].data = eeg_data[num_graph].slice(ini_pos, end_pos);
            // etiquetas   
            let labels_x = [];
            for (let i = ini_pos; i < end_pos; i++) labels_x.push(i);
            graphs_aux.graphs[num_graph].graph.config.data.labels = labels_x;
            graphs_aux.graphs[num_graph].graph.update("none");
        }
        graphs_aux.graphs[num_graph]["update_window_size"] = update_window_size;

        // evento a los botones izq y der
        right.onclick = () => {
            move_view("right");
        };
        left.onclick = () => {
            move_view("left");
        };
        // funciones para mover la gráfica
        graphs_aux.graphs[num_graph]["move_view"] = move_view;
    }

    // +++++++++++++++++++++++++
    //conexión con el topico ROS
    // +++++++++++++++++++++++++
    let actual_id = -1;
    async function ros_topic(msg) {
        let data = msg.eeg_data;
        let num_samples = msg.num_samples;
        num_channels = 16;
        time_capturing = msg.time_capturing;

        update_info_exp(msg.user_name, msg.id_pareto_front, msg.num_ind, msg.id_eeg_signal);
        // console.log(msg);
        // let id = msg.id;
        // if (actual_id != id) {
        //     actual_id = id;
        //     eeg_data = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []];
        // }   


        mutex.runExclusive(function () {
            // let is_new = false;
            // if (data.length > window_size) {
            //     eeg_data = [[], [], [], [], [], [], [], [], [], [], [], [], [], [], [], []];
            //     is_new = true;
            // }

            let size = eeg_data[0].length;

            //bloques a generar
            let num_w = parseInt(num_samples / slice_window);
            if (num_samples % slice_window > 0) num_w += 1;

            // tiempo
            let t;
            if (time_capturing == 0) t = 150;
            else t = time_capturing / num_w;

            let m_data = { ini: size, end: size + num_samples, size: num_samples, time: t, num_windows: num_w };
            time_update.push(m_data);

            //pasamos de 1D a 2D
            for (let i = 0; i < num_channels; i++) {
                let d = data.splice(0, num_samples);

                // d.push(-32000);
                // if (i == 0) d.push(35);
                // if (i == 1) d.push(90);
                // if (i == 2) d.push(190);
                // if (i == 3) d.push(290);
                // if (i > 3 && i < 12) d.push(490);
                // if (i >= 12) d.push(690);

                // d.push(950);
                if (msg.type_signal == "gui") {
                    //se eliminan las demas señales
                    eeg_data[i] = [];
                    // y se cargan las del experimento
                    eeg_data[i] = d;
                    // se actualiza la grafica del canal
                    graphs_aux.graphs[i].reset_view();
                } else {
                    //se concatenan datos del procesamiento anterior 
                    for (let k = 0; k < window_size * 0.3; k++) d.push(d[d.length - 1]);
                    eeg_data[i] = eeg_data[i].concat(d);
                    m_data.end = eeg_data[i].length;
                    change_mode("online");
                }
            }

            // if (is_new) {
            //     for (let num_graph = 0; num_graph < size_g; num_graph++) {
            //         graphs_aux.graphs[num_graph].reset_view();
            //     }
            // }
        });
    }

    // rosnodejs.nh.subscribe(topic_name, topic_type, ros_topic);
    rosnodejs.nh.subscribe("/gui_eeg_signals", "eeg_signal_acquisition/eeg_block", ros_topic);

}


function update_info_exp(user_name, id_pareto, num_ind, id_signal) {
    document.getElementById("selected_user").innerHTML = user_name;
    document.getElementById("id_pareto_front").innerHTML = id_pareto;
    document.getElementById("num_ind").innerHTML = num_ind;
    document.getElementById("id_signal").innerHTML = id_signal;
}


let cont = document.getElementById("container_real_time");
let graph_real_time = new new_graph(cont, "/gui_eeg_signals", "eeg_signal_acquisition/eeg_block");


function change_mode(type_view) {
    if (type_view == "online") {
        // se inicia la lectura online
        graph_real_time.run_read_signal();
    } else {
        // se detiene la lectura online
        graph_real_time.stop_update = true;
    }
}

// control

// // canales eeg
// let channel_data = [{
//     label: "canal1",
//     data: [],
//     lineTension: 0,
//     backgroundColor: "rgba(0,0,0,0)",
//     borderColor: "red",
//     yAxisID: 'chan1',
// },];