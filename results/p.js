
window.onload = function(){
	if(localStorage.input !== undefined){
		var tabox = document.getElementById('taIn');
		tabox.value = localStorage.input;
		generate();
	}
};

function generate(){
	var tabox = document.getElementById('taIn');
	localStorage.setItem("input", tabox.value + '');
	var lines = tabox.value.split('\n').map(line => line.split('\t'));
	lines = lines.filter(l => l.length == lines[0].length);
	
	let algorithms = lines.splice(0,1)[0];
	
	let genRow = (cols) => {
		let start = `<div style="text-align:center;border-right:1px solid black;float:left;width:${Math.floor(98/cols.length)}%;">`;
		console.log(start);
		let end = '</div>';
		return '<div style="display:inline-block;width:100%;">' +
			start + cols.join(end + start) + end
			'</div>';
	};
	
	let toDecPlaces = (x, n) => {
		return Math.round(x * Math.pow(10, n)) / Math.pow(10, n);
	};
	
	let means = algorithms.map((a,i) => {
		let sum = 0;
		for(let j = 0; j < lines.length; j++){
			sum += Number(lines[j][i]);
		}
		
		return toDecPlaces(1000 * sum / lines.length, 3);
	});
	
	let medians = algorithms.map((a,i) => {
		let data = [];
		for(let j = 0; j < lines.length; j++){
			let x = Number(lines[j][i]) * 1000;
			data.push(x);
		}
		data.sort();
		return toDecPlaces(data[Math.floor(data.length/2)],3);
	});
	
	let bests = lines.map(x => {
		let row = x.map((x,i) => { return {v:Number(x),i:i}; });
		row.sort((a,b) => a.v-b.v);
		return row[0];
	});
	
	let percentBestMatches = algorithms.map((a,i) => {
		let count = 0;
		for(let j = 0; j < lines.length; j++){
			if(bests[j].i == i) count++;
		}
		return toDecPlaces(100 * count / lines.length, 3);
	});
	
	
		
	document.getElementById('statsOut').innerHTML = genRow(['...'].concat(algorithms))
	+
	genRow(['mean:'].concat(means))
	+
	genRow(['median:'].concat(medians))
	+
	genRow(['%best:'].concat(percentBestMatches))
	;
	let nCs = n => {
		let r = '';
		for(let i = 0; i < n; i++) r += 'c';
		return r;
	};
	
	//\\textbf{frame} & \\textbf{FM-2D} & \\textbf{FM-3D} & \\textbf{ICP} & \\textbf{PCA} & \\textbf{FVR} & \\textbf{FVR-3D}
	
	let table = 
`\\begin{figure}
\\centering
\\begin{tabular}{${nCs(algorithms.length+1)}}
\\hline
${['frame'].concat(algorithms).map(name => '\\textbf{'+name+'}').join(' & ')}
\\\\ \\hline
${lines.map((l,ind) => (ind+1) + ' & ' + l.map(x=>toDecPlaces(x*1000,3)).join('&') + '\\\\').join('\n')}
\\end{tabular}

\\vspace{10mm}
\\centering
\\begin{tabular}{ccc}
\\hline
\\textbf{Algorithm} & \\textbf{Median Error $\\times$ 1000} & \\textbf{\\% best results}\\\\ \\hline
${algorithms.map((a,ind) => `${a}	& ${medians[ind]} & ~${percentBestMatches[ind]}\\%\\\\`).join('\n')}
\\end{tabular}

\\caption{Statistics for the Apartment Y-Axis Rotation Data Set}
\\label{tab:PET0ST}
\\end{figure} 
`;
	
	document.getElementById('taOut').innerHTML = table;
	
}