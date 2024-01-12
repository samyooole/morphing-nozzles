
from ocp import run_ocp
from displayMap import displayMap
import streamlit as st

def main():
    x,u,t=run_ocp()

    Map = displayMap(x,u,t)

    st.set_page_config(layout='wide')
    fig = Map.generate_visuals()

    st.plotly_chart(fig,use_container_width=True,height=800)



    
if __name__ == "__main__":
    main()