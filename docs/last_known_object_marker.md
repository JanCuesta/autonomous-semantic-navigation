# Last-known object visualization

The orange marker on `/semantic/perception/last_known_target_markers` shows the
last confirmed RGB-D position for the currently selected target class. It is
visualization only: it does not publish a target pose or validity and must not
be used to authorize navigation. Live perception can become invalid while the
marker remains visible. The marker is cleared when the selected class changes,
when the localizer detects a backwards depth timestamp (simulation clock reset),
and on localizer startup. It is retained only while the localizer process is
running; there is no persistence across restarts.

There is no map-change or AMCL reset notification wired to this localizer. If
the map or its coordinate reference is changed while the node is running,
restart the localizer before relying on the visualization. A clock reset is
recognized when a new depth image arrives with a timestamp more than 0.1 s
behind the previous image; a reset without subsequent depth images cannot be
detected by this mechanism.
