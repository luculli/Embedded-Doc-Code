## -----------------------------------------
## -- 3D Positioning based on POZYX Board
## -- (C) G. Luculli, 2018 - v0.5
## -----------------------------------------

## -- estimation of max/min
inflect = function(x, threshold = 1) {
	up   <- sapply(1:threshold, function(n) c(x[-(seq(n))], rep(NA, n)))
	down <-  sapply(-1:-threshold, function(n) c(rep(NA,abs(n)), x[-seq(length(x), length(x) - abs(n) + 1)]))
	a    <- cbind(x,up,down)
	list(minima = which(apply(a, 1, min) == a[,1]), maxima = which(apply(a, 1, max) == a[,1]))
}

## -- analysis of static positions
pozyx_analysis_static = function(fname) {
	ctr = length(fname);
	par(mfrow=c(ctr,3));
	
	## -- plot density
	res=data.frame(fname=character(), name=character(), est_bymax=numeric(), est_bymean=numeric(), stringsAsFactors = FALSE);
	trj = list();
	
	for(i in 1:ctr) {
		## -- read data
		dd = read.csv(fname[i], FALSE);
		
		## -- clean-up and formatting
		dd = dd[, c(3,4,5)];
		dd = na.omit(dd);
		names(dd) = c("x", "y", "z"); 
		
		## -- remember trajectory
		trj[[length(trj)+1]] = dd;
		
		## -- density plot
		tmp = density((dd$x)); mmpts = inflect(tmp$y); id_max = which(tmp$y[mmpts$maxima] == max(tmp$y[mmpts$maxima]));
		res[dim(res)[1]+1,] = c(fname[i], "x-sample", trunc(tmp$x[mmpts$maxima[id_max]]), trunc(mean(dd$x)));
		
		plot(tmp, main=paste(sep="", "x sample ", i)); grid(); abline(v = mean(dd$x), col = "red");
		points(tmp$x[mmpts$maxima[id_max]], tmp$y[mmpts$maxima[id_max]], pch=16, col="blue");
		
		tmp = density((dd$y)); mmpts = inflect(tmp$y); id_max = which(tmp$y[mmpts$maxima] == max(tmp$y[mmpts$maxima]));
		res[dim(res)[1]+1,] = c(fname[i], "y-sample", trunc(tmp$x[mmpts$maxima[id_max]]), trunc(mean(dd$y)));
		
		plot(density((dd$y)), main=paste(sep="", "y sample ", i)); grid(); abline(v = mean(dd$y), col = 2);
		points(tmp$x[mmpts$maxima[id_max]], tmp$y[mmpts$maxima[id_max]], pch=16, col="blue");
		
		tmp = density((dd$z)); mmpts = inflect(tmp$y); id_max = which(tmp$y[mmpts$maxima] == max(tmp$y[mmpts$maxima]));
		res[dim(res)[1]+1,] = c(fname[i], "z-sample", trunc(tmp$x[mmpts$maxima[id_max]]), trunc(mean(dd$z)));
		
		plot(density((dd$z)), main=paste(sep="", "z sample ", i)); grid(); abline(v = mean(dd$z), col = 2);
		points(tmp$x[mmpts$maxima[id_max]], tmp$y[mmpts$maxima[id_max]], pch=16, col="blue");
	}
	
	## -- ret results
	return(list(res=res, trj=trj));
} 

## -- comparison of position estimation based on mean vs max
comparison_mean_max = function(r) {
	ns = sort(unique(r$name));
	
	## -- compute stats error
	xHigh = c(); xLow = c(); yHigh = c(); yLow = c(); yVal = c();
	for(k in 1:length(ns)) {
		id = which(r$name == ns[k]);
		err_perc = signif(abs(as.numeric(r[id,]$est_bymax)-as.numeric(r[id,]$est_bymean))/as.numeric(r[id,]$est_bymax),2)*100;
		
		xHigh[k] = k; xLow[k] = k;
		yHigh[k] = max(err_perc); yLow[k] = min(err_perc); 
		yVal[k] = mean(err_perc);
	}
	
	## -- plot stats error
	plot(yVal, ylim=c(min(yLow), max(yHigh)), xlim = c(1, 0.5+max(length(ns))), ylab="%", xlab=NA, xaxt="n");
	arrows(xHigh, yHigh, xLow, yLow, col=2, angle=90, length=0.1, code=3);
	
	grid(); title(paste(sep="", "Error for each coordinate (%)"));
	text(1:length(ns), ns, pos=rep(4, length(ns)))
}

## -- plot dynamic trajectory
plot_dynamic_traj = function(r) {
	## -- get global ranges
	rgx=NULL; rgy=NULL;
	for(i in 1:length(r)) {
		rgx = range(r[[i]]$x, rgx);
		rgy = range(r[[i]]$y, rgy);
	}
	
	## -- plot overall trajectories
	plot(0, xlim=rgx, ylim=rgy, xlab="x (mm)", ylab="y (mm)", main="Detected trajectory"); grid();
	colors = c("green", "red", "blue", "orange", "black", "yellow", "cyan");
	
	for(i in 1:length(r)) {
		par(new=T);
		plot(r[[i]]$x, r[[i]]$y, col=colors[i], xlim=rgx, ylim=rgy, yaxt='n', xaxt='n', xlab='', ylab='');
	}
}

## -- gui
gui_ipa = function() {
	## -- General params
	idir = list.dirs('.', recursive=FALSE);
	idir = idir[grep("data-", idir)];
	
	## -- Setup GUI (1)
	library(gWidgets); options(guiToolkit="RGtk2"); 
	
	win = gwindow("Indoor Position Analysis (static & dynamic)", width=1200, height=600, expand = TRUE, fill=TRUE, visible=TRUE);
	on.exit({ visible(win) = TRUE; svalue(notebook) = 1; });
	
	group = ggroup(horizontal= TRUE, container = win, expand=TRUE);
	
	frame = gframe("", container=group, horizontal=FALSE, expand = FALSE);
	lyt = glayout (cont = frame, spacing=5, expand = TRUE);
	widget_list = list(); 
	
	widget_list$tab = NULL; widget_list$txt = NULL;
	
	lyt[1,1] = "  idir "; lyt[1,2] = (widget_list$cb_idir = gcombobox(idir, cont=lyt));  
	lyt[2,1] = "  h/v "; lyt[2,2] = (widget_list$cb_mode = gcombobox(c("h", "v"), cont=lyt));  
	lyt[3,1:2] =  gseparator(container=lyt);
	
	notebook = gnotebook(container=group, expand=TRUE, fill=TRUE);
	sbar = gstatusbar(text = "", container = win);
	
	## -- Setup GUI (2)
	gg = list(name=list(), des=list(), id=list());
	
	gg$name[[1]] =  "Analysis";
	gg$des[[1]] = ggroup(horizontal= TRUE, space=6, container = notebook, expand=TRUE, fill=TRUE, label=gg$name[[1]]);
	gg$id[[1]] =  max(dev.list());
	
	gg$name[[2]] =  "Graph 1";
	gg$des[[2]] = ggraphics(width=800, height=300, container=gg$des[[1]]);
	gg$id[[2]] =  max(dev.list());
	
	gg$name[[3]] =  "Graph 2";
	gg$des[[3]] = ggraphics(width=100, height=300, container=gg$des[[1]]);
	gg$id[[3]] =  max(dev.list());
	
	visible(win) = TRUE;
	
	## -- GUI handlers
	addHandlerChanged(widget_list$cb_idir , handler = function(h, ...) {
				myplot();
			});
	
	addHandlerChanged(widget_list$cb_mode , handler = function(h, ...) {
				myplot();
			});
	
	myplot = function() {
		## -- recover files list
		fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_fixed_point_", svalue(widget_list$cb_mode), "_balgo_"), full.names=T);

		if (length(fname) == 0)  
			fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_fixed_point_", svalue(widget_list$cb_mode), "_lsalgo_"), full.names=T);

		if (length(fname) == 0)  
			fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_fixed_point_", svalue(widget_list$cb_mode), "_tralgo_"), full.names=T);
		
		if (length(fname) == 0)  
			fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_dynamic_", svalue(widget_list$cb_mode), "_lsalgo_"), full.names=T);
		
		if (length(fname) == 0)  
			fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_dynamic_", svalue(widget_list$cb_mode), "_balgo_"), full.names=T);

		if (length(fname) == 0)  
			fname = list.files(svalue(widget_list$cb_idir), paste(sep="", "data_dynamic_", svalue(widget_list$cb_mode), "_tralgo_"), full.names=T);
		
		## -- plot position analysis + error statistics
		dev.set(gg$id[[2]]);
		
		if (length(fname) != 0) {
			## -- do analysis and plot distributions
			r_balgo = pozyx_analysis_static(fname);
			
			## -- plot beacons analysis
			dev.set(gg$id[[3]]); par(mfrow=c(2,1));
			comparison_mean_max(r_balgo$res);
			
			## -- plot trajectories
			if (length(r_balgo$trj)) plot_dynamic_traj(r_balgo$trj);
			
			## -- done
			svalue(sbar) = "done!";
		}
		else {
			svalue(sbar) = "data not found!";
		}
	}
	
}

## -- run
gui_ipa();
