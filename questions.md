* Measurement update of Radar:
- why can't I just convert the measurement into kartesian space and use the converted data in the measurement update function?

* Q
- Can't I just add the acceleration to the state vector? I know I can't measure it with a sensor, but I can calculate it in each prediction step ((v2 - v1) / dt). Where does the filter break if I'd do it this way?

* Simulator
- a [step] button would be nice
- uses two cores while idling and doing nothing ..
- a line-plot would be better as visualization (at least for the filtered position)