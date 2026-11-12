# Bags

Bag files are not included in this repository because of GitHub file size limitations. This folder is where the launch files look for them: place here the bags you want to run the localization system on (simulated or real), and pass their path relative to this folder with `bag_name:=...`.

To get the real-world dataset from the paper, download `mr_radio_dataset.tar.gz` from [Harvard Dataverse](https://dataverse.harvard.edu/dataset.xhtml?persistentId=doi:10.7910/DVN/KNKWMJ) and extract it here, so that you get `experiment1_gt_v2/`. See [Running the real-world dataset](../../docs/dataset.md).
