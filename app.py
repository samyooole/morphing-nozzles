
from ocp import run_ocp
from displayMap import displayMap
import streamlit as st
import numpy as np

def main():
    #x,u,t=run_ocp()
    x = np.load('x.npy')
    u = np.load('u.npy')
    t = np.load('t.npy')

    Map = displayMap(x,u,t)

    st.set_page_config(page_title='Optimal control of orbital path', layout='wide')
    fig = Map.generate_visuals()

    st.plotly_chart(fig,use_container_width=True,height=800)


    
if __name__ == "__main__":
    main()