let param_neuro = {}
param_neuro["type"] = "number";
param_neuro["id"] = "simulation_time";
param_neuro["text"] = "Tiempo de simulación";
let sim_time = create_input_param(param_neuro);
sim_time.load_param();



param_neuro = {}
param_neuro["type"] = "number";
param_neuro["id"] = "contact_thresold";
param_neuro["text"] = "Umbral de contacto";
let contact_t = create_input_param(param_neuro);
contact_t.load_param();


let container_neuro = document.getElementById("container_neurocontroller");
container_neuro.appendChild(sim_time);
container_neuro.appendChild(contact_t);
